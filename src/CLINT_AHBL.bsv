// Copyright (c) 2016-2019 Bluespec, Inc. All Rights Reserved

package CLINT_AHBL;

// ================================================================
// This package implements an AHB-L slave IP with two pieces of RISC-V
// functionality that are unrelated except in that they generate local
// interrupts for the Core.
//
// These are also known as CLINT (Core-Local Interruptor) in other RISC-V systems.
//
// - real-time timer:
//     Two 64-bit memory-mapped registers (rg_time and rg_timecmp).
//     Delivers an external interrupt whenever rg_timecmp <= rg_time.
//     The timer interrupt is cleared when rg_timecmp is written.
//     Can be used for the RISC-V v1.10 Privilege Spec 'mtime' and
//     'mtimecmp', and provides a memory-mapped API to access them.
//
//     Offset/Size        Name        Function
//     'h_4000/8 Bytes    mtimecmp    R/W the hart0 mtimecmp  register
//     'h_BFF8/8 Bytes    mtime       R/W the mtime     register
//
// - Memory-mapped location for software interrupts.
//
//     Offset/Size        Name        Function
//     'h_0000/8 Bytes    msip        R/W Writing LSB=1 generates a software interrupt to hart0
//
// ----------------
// This slave IP can be attached to fabrics with 32b-wide data channels.
//    (NOTE: this is the width of the fabric, which can be chosen
//      independently of the native width of a CPU master on the
//      fabric (such as RV32/RV64 for a RISC-V CPU).
// When attached to 32b-wide fabric, 64-bit locations must be
// read/written in two 32b transaction, once for the lower 32b and
// once for the upper 32b.
//
// Some of the 'truncate()'s and 'zeroExtend()'s below are no-ops but
// necessary to satisfy type-checking.
//
// NOTE: This module does not conduct address range checks. It is
// expected that address range checks are the responsibility of the
// system AHB-L decoder.
// ================================================================

// BSV library imports

import  FIFOF        :: *;
import  GetPut       :: *;
import  ClientServer :: *;
import  ConfigReg    :: *;
import  Vector       :: *;

// ----------------
// BSV additional libs

import Cur_Cycle     :: *;

// ================================================================
// Project imports

// Main fabric
import AHBL_Types    :: *;
import AHBL_Defs     :: *;
import Fabric_Defs   :: *;    // for Wd_Id, Wd_Addr, Wd_Data, Wd_User

// ================================================================
// Local constants and types

// Module state
typedef enum { RST, RDY, RSP, ERR1, ERR2 } AHBL_Tgt_State deriving (Bits, Eq, FShow);
// ================================================================
// Interface

interface CLINT_AHBL_IFC;
   // Main Fabric Reqs/Rsps
   interface AHBL_Slave_IFC #(AHB_Wd_Data) fabric;

   // Timer interrupt
   // True/False = set/clear interrupt-pending in CPU's MTIP
   (* always_ready *)
   method Bool  timer_interrupt_pending;

   // Software interrupt
   (* always_ready *)
   method Bool  sw_interrupt_pending;
endinterface

// ================================================================
(*doc="This module does not conduct address range checks."*)
(*doc="It is assumed that address range checks are the"*)
(*doc="responsibility of the system AHB-L decoder."*)
(* synthesize *)
module mkCLINT_AHBL (CLINT_AHBL_IFC);

   // Verbosity: 0: quiet; 1: reset; 2: timer interrupts, all reads and writes
   Bit #(2) verbosity = 0;

   // Base and limit addrs for this memory-mapped block.
   Fabric_Addr addr_mask = 'hffff;

   // ----------------
   // Timer registers

   Reg #(Bit #(64)) crg_time [2]    <- mkCReg (2, 1);
   Reg #(Bit #(64)) crg_timecmp [2] <- mkCReg (2, 0);

   Reg #(Bool) rg_mtip <- mkReg (True);

   // ----------------
   // Software-interrupt registers

   Reg #(Bool) rg_msip <- mkRegU;

   // Software interrupt queue
   FIFOF #(Bool) f_sw_interrupt_req <- mkFIFOF;

   // ----------------
   // AHB-Lite signals and registers

   // Inputs
   Wire #(Bool)            w_hsel      <- mkBypassWire;
   Wire #(Bool)            w_hready_in <- mkBypassWire;
   Wire #(AHB_Fabric_Addr) w_haddr     <- mkBypassWire;
   Wire #(AHBL_Burst)      w_hburst    <- mkBypassWire;
   Wire #(Bool)            w_hmastlock <- mkBypassWire;
   Wire #(AHBL_Prot)       w_hprot     <- mkBypassWire;
   Wire #(AHBL_Size)       w_hsize     <- mkBypassWire;
   Wire #(AHBL_Trans)      w_htrans    <- mkBypassWire;
   Wire #(AHB_Fabric_Data) w_hwdata    <- mkBypassWire;
   Wire #(Bool)            w_hwrite    <- mkBypassWire;

   // Outputs
   Reg  #(Bool)            rg_hready    <- mkReg(True);
   Reg  #(AHBL_Resp)       rg_hresp     <- mkReg(AHBL_OKAY);
   Reg  #(AHB_Fabric_Data) rg_hrdata    <- mkRegU;

   Reg #(AHB_Fabric_Addr)  rg_haddr     <- mkRegU;
   Reg #(AHBL_Size)        rg_hsize     <- mkRegU;
   Reg #(AHBL_Trans)       rg_htrans    <- mkRegU;
   Reg #(Bool)             rg_hwrite    <- mkRegU;

   Reg #(AHBL_Tgt_State)   rg_state  <- mkReg (RST);

   // ================================================================
   // BEHAVIOR
   // ----------------
   // Address Checks

   // Is the address okay? Use the raw address from the bus as this check is done
   // in the first phase.
   let addr_is_ok = fn_ahbl_is_aligned (w_haddr[1:0], w_hsize);

   // ----------------------------------------------------------------
   // Reset

   rule rl_reset (rg_state == RST);
      crg_time [1]    <= 1;
      crg_timecmp [1] <= 0;
      rg_mtip         <= True;
      rg_msip         <= False;

      // ready the fabric interface ...
      rg_state        <= RDY;
      rg_hready       <= True;
      rg_hresp        <= AHBL_OKAY;

      if (verbosity != 0)
         $display ("%06d:[D]:%m.rl_reset", cur_cycle);
   endrule

   // ----------------------------------------------------------------
   // Keep time and generate interrupt

   // Increment time, but saturate, do not wrap-around
   (* fire_when_enabled, no_implicit_conditions *)
   rule rl_tick_timer (   (rg_state == RDY)
                       && (crg_time [0] != '1));

      crg_time [0] <= crg_time [0] + 1;
   endrule

   // Compare and generate timer interrupt request

   Bool new_mtip = (crg_time [0] >= crg_timecmp [0]);

   rule rl_compare ((rg_state == RDY)
                    && (rg_mtip != new_mtip));

      rg_mtip <= new_mtip;
      if (verbosity > 1)
         $display ("%6d:[D]:%m.rl_compare: new MTIP = %0d, time = %0d, timecmp = %0d",
                   cur_cycle, new_mtip, crg_time [0], crg_timecmp [0]);
   endrule

   // ================================================================
   // Bus interface for reading/writing control/status regs
   // Relative-address map is same as 'SiFive U54-MC Core Complex Manual v1p0'.
   // Accesses are 4-bytes wide, even though bus may be 64b wide.
   
   function Action fa_ahbl_error;
      action
         rg_state    <= ERR1;
         rg_hready   <= False;
         rg_hresp    <= AHBL_ERROR;
      endaction
   endfunction 

   (* fire_when_enabled, no_implicit_conditions *)
   rule rl_new_req (   (rg_state == RDY)
                    && (w_hsel && w_hready_in && (w_htrans == AHBL_NONSEQ)));

      // Register fresh address-and-control inputs
      rg_haddr     <= w_haddr;
      rg_hsize     <= w_hsize;
      rg_htrans    <= w_htrans;
      rg_hwrite    <= w_hwrite;

      if (addr_is_ok) begin
         rg_state    <= RSP;
         rg_hresp    <= AHBL_OKAY;

         // Immediate responses are not possible because error
         // checking requires an extra cycle for timing reasons
         rg_hready   <= False;
      end

      // Error case (two cycle response)
      else begin
         fa_ahbl_error;
      end

      if (verbosity != 0)
         $display ("%06d:[D]:%m.rl_new_req: haddr 0x%08h"
         , cur_cycle, w_haddr, fshow (w_hsize), " hwrite %0d htrans ", w_hwrite, fshow (w_htrans));
   endrule

   // ----------------------------------------------------------------
   // Handle memory-mapped write requests

   let byte_addr = rg_haddr & addr_mask;
   rule rl_wr_req ((rg_state == RSP) && (rg_hwrite));
      let wdata = w_hwdata;
      let werr = False;

      if (verbosity > 1) begin
         $display ("%06d:[D]:%m.rl_wr_req: rg_mtip = %0d", cur_cycle, rg_mtip);
         $display ("            (byte_addr 0x%08h) (wdata 0x%08h)", byte_addr, wdata);
      end

      // MSIP
      if (byte_addr == 'h_0000) begin
         Bool new_msip = (wdata [0] == 1'b1);
         if (rg_msip != new_msip) begin
            rg_msip <= new_msip;
            if (verbosity > 1)
               $display ("            new MSIP = %0d", new_msip);
         end
      end

      // The following ALIGN4B writes are only needed for 32b fabrics
      else if (byte_addr == 'h_0004) begin
         // MSIPH
         noAction;    // upper 32 bits wired to 0
      end

      // 64-bit mtimecmp register (lower-half)
      else if (byte_addr == 'h_4000) begin   // byte_addr 4000
         Vector # (2, Bit #(32)) old_timecmp = unpack (crg_timecmp [1]);
         Bit #(32) new_timecmp_L = fn_ahbl_update_wdata (  byte_addr[1:0]
                                                     , rg_hsize
                                                     , old_timecmp[0]
                                                     , wdata);
         crg_timecmp [1] <= {old_timecmp[1], new_timecmp_L};

         if (verbosity > 1) begin
            $display ("            Writing MTIMECMP");
            $display ("                old MTIMECMP         = 0x%016h", crg_timecmp[1]);
            $display ("                new MTIMECMP         = 0x%016h", {old_timecmp[1], new_timecmp_L});
            $display ("                cur MTIME            = 0x%016h", crg_time [1]);
         end
      end

      // 64-bit mtimecmp register (upper-half). For 32-bit fabrics only
      else if (byte_addr == 'h_4004) begin   // byte_addr 4004
         Vector # (2, Bit #(32)) old_timecmp = unpack (crg_timecmp [1]);
         Bit #(32) new_timecmp_H = fn_ahbl_update_wdata (  byte_addr[1:0]
                                                     , rg_hsize
                                                     , old_timecmp[1]
                                                     , wdata);
         crg_timecmp [1] <= {new_timecmp_H, old_timecmp[0]};

         if (verbosity > 1) begin
            $display ("            Writing MTIMECMP");
            $display ("                old MTIMECMP         = 0x%016h", crg_timecmp[1]);
            $display ("                new MTIMECMP         = 0x%016h", {new_timecmp_H, old_timecmp[0]});
            $display ("                cur MTIME            = 0x%016h", crg_time [1]);
         end
      end

      // 64-bit MTIME register (lower-half)
      else if (byte_addr == 'h_BFF8) begin   // byte_addr BFF8
         Vector #(2, Bit #(32)) old_time = unpack (crg_time [1]);
         Bit #(32) new_time_L = fn_ahbl_update_wdata (  byte_addr [1:0]
                                                  , rg_hsize
                                                  , old_time[0]
                                                  , wdata);
         crg_time [1] <= {old_time[1], new_time_L};

         if (verbosity > 1) begin
            $display ("    Writing MTIME");
            $display ("        old MTIME = 0x%016h", crg_time [1]);
            $display ("        new MTIME = 0x%016h", {old_time[1], new_time_L});
         end
      end

      // 64-bit MTIME register (upper-half)
      else if (byte_addr == 'h_BFFC) begin   // byte_addr BFFC
         Vector #(2, Bit #(32)) old_time = unpack (crg_time [1]);
         Bit #(32) new_time_H = fn_ahbl_update_wdata (  byte_addr [1:0]
                                                      , rg_hsize
                                                      , old_time[1]
                                                      , wdata);
         crg_time [1] <= {new_time_H, old_time[0]};

         if (verbosity > 1) begin
            $display ("    Writing MTIME");
            $display ("        old MTIME = 0x%016h", crg_time [1]);
            $display ("        new MTIME = 0x%016h", {new_time_H, old_time[0]});
         end
      end

      // Error condition bad address
      else begin
         werr = True;
         $display ("%06d:[E]:%m.rl_wr_req: unrecognized addr: 0x%08h"
            , cur_cycle, byte_addr);
      end

      if (werr) fa_ahbl_error;
      else begin
         rg_hready <= True;
         rg_state  <= RDY;
      end
   endrule

   // ----------------------------------------------------------------
   // Handle 'memory'-read requests

   rule rl_rd_req ((rg_state == RSP) && (!rg_hwrite));
      if (verbosity > 1) begin
         $display ("%06d:[D]:%m.rl_rd_req: rg_mtip = %0d", cur_cycle, rg_mtip);
         $display ("            (byte_addr 0x%08h)", byte_addr);
      end

      Bit #(32) rdata = ?;
      let rerr = False;

      if (byte_addr == 'h_0000)
         // MSIP
         rdata = zeroExtend (rg_msip ? 1'b1 : 1'b0);

      else if (byte_addr == 'h_4000)
         // MTIMECMP
         rdata = truncate (crg_timecmp [0]);    // truncates for 32b fabrics

      else if (byte_addr == 'h_BFF8)
         // MTIME
         rdata = truncate (crg_time [0]);       // truncates for 32b fabrics

      // The following ALIGN4B reads are only needed for 32b fabrics
      else if (byte_addr == 'h_0004)
         // MSIPH
         rdata = 0;

      else if (byte_addr == 'h_4004) begin
         // MTIMECMPH
         Bit #(64) x64 = crg_timecmp [0];
         rdata = x64[63:32];
      end

      else if (byte_addr == 'h_BFFC) begin
         // MTIMEH
         Bit #(64) x64 = crg_time [0];
         rdata = x64[63:32];
      end

      else begin
         rerr = True;
         $display ("%06d:[E]:%m.rl_rd_req: unrecognized addr: 0x%08h"
            , cur_cycle, byte_addr);
      end

      if (rerr) fa_ahbl_error;
      else begin
         rg_hready <= True;
         rg_state  <= RDY;
         rg_hrdata <= rdata;
      end
   endrule

   rule rl_idle (   (rg_state == RDY)
                 && (w_hsel && (w_htrans == AHBL_IDLE)));
      rg_hready <= True;
   endrule

   rule rl_error1 (rg_state == ERR1);
      rg_state  <= ERR2;
      rg_hready <= True;
   endrule

   rule rl_error2 (rg_state == ERR2);
      rg_state  <= RDY;
      rg_hready <= True;
      rg_hresp  <= AHBL_OKAY;
   endrule


   // ================================================================
   // INTERFACE

   // Memory-mapped access: fabric interface
   interface AHBL_Slave_IFC fabric;
      // ----------------
      // Inputs
      method Action hsel (Bool sel);
         w_hsel <= sel;
      endmethod

      method Action hready (Bool hready_in);
         w_hready_in <= hready_in;
      endmethod

      method Action haddr (AHB_Fabric_Addr addr);
         w_haddr <= addr;
      endmethod

      method Action hburst (AHBL_Burst burst);
         w_hburst <= burst;
      endmethod

      method Action hmastlock (Bool mastlock);
         w_hmastlock <= mastlock;
      endmethod

      method Action hprot (AHBL_Prot prot);
         w_hprot <= prot;
      endmethod

      method Action hsize (AHBL_Size size);
         w_hsize <= size;
      endmethod

      method Action htrans (AHBL_Trans trans);
         w_htrans <= trans;
      endmethod

      method Action hwdata(AHB_Fabric_Data data);
         w_hwdata <= data;
      endmethod

      method Action hwrite (Bool write);
         w_hwrite <= write;
      endmethod

      // ----------------
      // Outputs

      method Bool       hreadyout = rg_hready;
      method AHBL_Resp  hresp     = rg_hresp;
      method AHB_Fabric_Data  hrdata    = rg_hrdata;
   endinterface

   // Timer interrupt
   method Bool timer_interrupt_pending = rg_mtip;

   // Software interrupt
   method Bool sw_interrupt_pending = rg_msip;
endmodule

// ================================================================

endpackage
