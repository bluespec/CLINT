// Copyright (c) 2016-2019 Bluespec, Inc. All Rights Reserved

package CLINT_AXI4;

// ================================================================
// This package implements an AXI4 slave IP with two pieces of RISC-V
// functionality that are unrelated except in that they generate local
// interrupts for the Core.
//
// These are also known as CLINT (Core-Local Interruptor) in other RISC-V systems.
//
// - real-time timer:
//     Two 64-bit memory-mapped registers (rg_time and rg_timecmp).
//     Delivers an external interrupt whenever rg_timecmp >= rg_time.
//     The timer is cleared when rg_timecmp is written.
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
// This slave IP can be attached to fabrics with 32b- or 64b-wide data channels.
//    (NOTE: this is the width of the fabric, which can be chosen
//      independently of the native width of a CPU master on the
//      fabric (such as RV32/RV64 for a RISC-V CPU).
// When attached to 32b-wide fabric, 64-bit locations must be
// read/written in two 32b transaction, once for the lower 32b and
// once for the upper 32b.
//
// Some of the 'truncate()'s and 'zeroExtend()'s below are no-ops but
// necessary to satisfy type-checking.
// ================================================================

// BSV library imports

import  FIFOF         :: *;
import  GetPut        :: *;
import  ClientServer  :: *;
import  ConfigReg     :: *;

// ----------------
// BSV additional libs

import Cur_Cycle  :: *;
import GetPut_Aux :: *;
import Semi_FIFOF :: *;
import ByteLane   :: *;

// ================================================================
// Project imports

// Main fabric
import SoC_Map       :: *;
import AXI4_Types   :: *;
import AXI4_Fabric  :: *;
import Fabric_Defs  :: *;    // for Wd_Id, Wd_Addr, Wd_Data, Wd_User

// ================================================================
// Local constants and types

// Module state
typedef enum {MODULE_STATE_START, MODULE_STATE_READY } Module_State
deriving (Bits, Eq, FShow);

// ================================================================
// Interface

interface CLINT_AXI4_IFC;
   // Memory-mapped access
   interface AXI4_Slave_IFC #(Wd_Id, Wd_Addr, Wd_Data, Wd_User) axi4;

   // Timer interrupt
   // True/False = set/clear interrupt-pending in CPU's MTIP
   method Bool  timer_interrupt_pending;

   // Software interrupt
   method Bool  sw_interrupt_pending;
endinterface

// ================================================================

(* synthesize *)
module mkCLINT_AXI4 (CLINT_AXI4_IFC);

   // Verbosity: 0: quiet; 1: reset; 2: timer interrupts, all reads and writes
   Bit #(2) verbosity = 0;

   Reg #(Module_State) rg_state     <- mkReg (MODULE_STATE_START);

   // Base and limit addrs for this memory-mapped block.
   SoC_Map_IFC soc_map <- mkSoC_Map;

   // ----------------
   // Memory-mapped access

   // Connector to AXI4 fabric
   AXI4_Slave_Xactor_IFC #(Wd_Id, Wd_Addr, Wd_Data, Wd_User) slave_xactor <- mkAXI4_Slave_Xactor;

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

   // ================================================================
   // BEHAVIOR

   // ----------------------------------------------------------------
   // Reset

   rule rl_reset (rg_state == MODULE_STATE_START);
      slave_xactor.reset;
      f_sw_interrupt_req.clear;

      rg_state        <= MODULE_STATE_READY;
      crg_time [1]    <= 1;
      crg_timecmp [1] <= 0;
      rg_mtip         <= True;
      rg_msip         <= False;

      if (verbosity != 0)
	 $display ("%06d:[D]:%m.rl_reset", cur_cycle);
   endrule

   // ----------------------------------------------------------------
   // Keep time and generate interrupt

   // Increment time, but saturate, do not wrap-around
   (* fire_when_enabled, no_implicit_conditions *)
   rule rl_tick_timer (   (rg_state == MODULE_STATE_READY)
		       && (crg_time [0] != '1));

      crg_time [0] <= crg_time [0] + 1;
   endrule

   // Compare and generate timer interrupt request

   Bool new_mtip = (crg_time [0] >= crg_timecmp [0]);

   rule rl_compare ((rg_state == MODULE_STATE_READY)
		    && (rg_mtip != new_mtip));

      rg_mtip <= new_mtip;
      if (verbosity > 1)
	 $display ("%0d: Near_Mem_IO_AXI4.rl_compare: new MTIP = %0d, time = %0d, timecmp = %0d",
		   cur_cycle, new_mtip, crg_time [0], crg_timecmp [0]);
   endrule

   // ----------------------------------------------------------------
   // Handle 'memory'-read requests

   rule rl_process_rd_req (rg_state == MODULE_STATE_READY);

      let rda <- pop_o (slave_xactor.o_rd_addr);
      if (verbosity > 1) begin
	 $display ("%06d:[D]:%m.rl_process_rd_req: rg_mtip = %0d", cur_cycle, rg_mtip);
	 $display ("    ", fshow (rda));
      end

      let        byte_addr = rda.araddr - soc_map.m_clint_addr_base;
      Bit #(64)  rdata = 0;
      AXI4_Resp  rresp = axi4_resp_okay;

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
	 if (valueOf (Wd_Data) == 32)
	    x64 = { 0, x64 [63:32] };
	 rdata = zeroExtend (x64);    // extends for 64b fabrics
      end

      else if (byte_addr == 'h_BFFC) begin
	 // MTIMEH
	 Bit #(64) x64 = crg_time [0];
	 if (valueOf (Wd_Data) == 32)
	    x64 = { 0, x64 [63:32] };
	 rdata = zeroExtend (x64);    // extends for 64b fabrics
      end

      else
	 rresp = axi4_resp_decerr;

      if (rresp != axi4_resp_okay) begin
	 $display ("%06d:[E]:%m.rl_process_rd_req: unrecognized addr", cur_cycle);
	 $display ("            ", fshow (rda));
      end

      // Send read-response to bus
      Fabric_Data x = truncate (rdata);
      let rdr = AXI4_Rd_Data {rid:   rda.arid,
			      rdata: x,
			      rresp: rresp,
			      rlast: True,
			      ruser: rda.aruser};
      slave_xactor.i_rd_data.enq (rdr);

      if (verbosity > 1) begin
	 $display ("%06d:[D]:%m.rl_process_rd_req", cur_cycle);
	 $display ("            ", fshow (rda));
	 $display ("            ", fshow (rdr));
      end
   endrule

   // ----------------------------------------------------------------
   // Handle 'memory'-write requests

   rule rl_process_wr_req (rg_state == MODULE_STATE_READY);

      let wra <- pop_o (slave_xactor.o_wr_addr);
      let wrd <- pop_o (slave_xactor.o_wr_data);
      if (verbosity > 1) begin
	 $display ("%06d:[D]:%m.rl_process_wr_req: rg_mtip = %0d", cur_cycle, rg_mtip);
	 $display ("    ", fshow (wra));
	 $display ("    ", fshow (wrd));
      end

      Bit #(64) wdata     = zeroExtend (wrd.wdata);
      Bit #(8)  wstrb     = zeroExtend (wrd.wstrb);
      Bit #(8)  data_byte = wdata [7:0];

      let        byte_addr = wra.awaddr - soc_map.m_clint_addr_base;
      AXI4_Resp  bresp     = axi4_resp_okay;

      if (byte_addr == 'h_0000) begin
	 // MSIP
	 Bool new_msip = (wdata [0] == 1'b1);
	 if (rg_msip != new_msip) begin
	    rg_msip <= new_msip;
	    if (verbosity > 1)
	       $display ("    new MSIP = %0d", new_msip);
	 end
      end

      else if (byte_addr == 'h_4000) begin
	 // MTIMECMP
	 Bit #(64) old_timecmp = crg_timecmp [1];
	 Bit #(64) new_timecmp = fn_update_strobed_bytes (old_timecmp,
							  zeroExtend (wdata),
							  zeroExtend (wstrb));
	 crg_timecmp [1] <= new_timecmp;

	 if (verbosity > 1) begin
	    $display ("    Writing MTIMECMP");
	    $display ("        old MTIMECMP         = 0x%0h", old_timecmp);
	    $display ("        new MTIMECMP         = 0x%0h", new_timecmp);
	    $display ("        cur MTIME            = 0x%0h", crg_time [1]);
	    $display ("        new MTIMECMP - MTIME = 0x%0h", new_timecmp - crg_time [1]);
	 end
      end

      else if (byte_addr == 'h_BFF8) begin
	 // MTIME
	 Bit #(64) old_time = crg_time [1];
	 Bit #(64) new_time = fn_update_strobed_bytes (old_time,
						       zeroExtend (wdata),
						       zeroExtend (wstrb));
	 crg_time [1] <= new_time;

	 if (verbosity > 1) begin
	    $display ("    Writing MTIME");
	    $display ("        old MTIME = 0x%0h", old_time);
	    $display ("        new MTIME = 0x%0h", new_time);
	 end
      end

      // The following ALIGN4B writes are only needed for 32b fabrics
      else if (byte_addr == 'h_0004) begin
	 // MSIPH
	 noAction;    // upper 32 bits wired to 0
      end

      else if (byte_addr == 'h_4004) begin
	 // MTIMECMPH
	 Bit #(64) old_timecmp = crg_timecmp [1];
	 Bit #(64) x64      = zeroExtend (wdata);
	 Bit #(8)  x64_strb = zeroExtend (wstrb);
	 if (valueOf (Wd_Data) == 32) begin
	    x64      = { x64 [31:0], 0 };
	    x64_strb = { x64_strb [3:0], 0 };
	 end
	 Bit #(64) new_timecmp = fn_update_strobed_bytes (old_timecmp, x64, x64_strb);
	 crg_timecmp [1] <= new_timecmp;

	 if (verbosity > 1) begin
	    $display ("    Writing MTIMECMP");
	    $display ("        old MTIMECMP         = 0x%0h", old_timecmp);
	    $display ("        new MTIMECMP         = 0x%0h", new_timecmp);
	    $display ("        cur MTIME            = 0x%0h", crg_time [1]);
	    $display ("        new MTIMECMP - MTIME = 0x%0h", new_timecmp - crg_time [1]);
	 end
      end

      else if (byte_addr == 'h_BFFC) begin
	 // MTIMEH
	 Bit #(64) old_time = crg_time [1];
	 Bit #(64) x64      = zeroExtend (wdata);
	 Bit #(8)  x64_strb = zeroExtend (wstrb);
	 if (valueOf (Wd_Data) == 32) begin
	    x64      = { x64 [31:0], 0 };
	    x64_strb = { x64_strb [3:0], 0 };
	 end
	 Bit #(64) new_time = fn_update_strobed_bytes (old_time, x64, x64_strb);
	 crg_time [1] <= new_time;

	 if (verbosity > 1) begin
	    $display ("    Writing MTIME");
	    $display ("        old MTIME = 0x%0h", old_time);
	    $display ("        new MTIME = 0x%0h", new_time);
	 end
      end

      else
	 bresp = axi4_resp_decerr;

      if (bresp != axi4_resp_okay) begin
	 $display ("%06d:[E]:%m.rl_process_wr_req: unrecognized addr", cur_cycle);
	 $display ("            ", fshow (wra));
	 $display ("            ", fshow (wrd));
      end

      // Send write-response to bus
      let wrr = AXI4_Wr_Resp {bid:   wra.awid,
			      bresp: bresp,
			      buser: wra.awuser};
      slave_xactor.i_wr_resp.enq (wrr);

      if (verbosity > 1) begin
	 $display ("%06d:[D]:%m.AXI4.rl_process_wr_req", cur_cycle);
	 $display ("            ", fshow (wra));
	 $display ("            ", fshow (wrd));
	 $display ("            ", fshow (wrr));
      end
   endrule

   // ================================================================
   // INTERFACE

   // Memory-mapped access
   interface  axi4 = slave_xactor.axi_side;

   // Timer interrupt
   method Bool timer_interrupt_pending = rg_mtip;

   // Software interrupt
   method Bool sw_interrupt_pending = rg_msip;

endmodule

// ================================================================

endpackage
