package axi4lite_axi4lite;
/*=== Project imports ====*/
import AXI4_Lite_Fabric::*;
import AXI4_Lite_Types::*;
import AXI4_Fabric::*;
import AXI4_Types ::*;
import Semi_FIFOF	::*;
import bridge_types ::*;
import bridge_functions ::*;
import Connectable ::*;
/*======================*/
/*=== Package imports ===*/
import Clocks::*;
import BUtils ::*;
/*=======================*/
instance ConnectableClocks#(AXI4_Lite_Master_IFC#(paddr,data_width_in,userspace),   
																				AXI4_Lite_Slave_IFC#(paddr,data_width_out,userspace))
																		provisos(Max#(data_width_in, data_width_out, data_width_max),
																						 Min#(data_width_in, data_width_out, data_width_min),
																						 Mul#(data_ratio, data_width_min, data_width_max),
																						 Add#(i__, data_width_min, data_width_out),
																						 Mul#(j__, data_width_out, data_width_max),
																						 Add#(k__, data_width_in, data_width_max));
	module mkConnectionClocks#(AXI4_Lite_Master_IFC#(paddr,data_width_in,userspace) bus1, 
																															Clock fast_clock, Reset fast_reset, 
																AXI4_Lite_Slave_IFC#(paddr,data_width_out,userspace) bus2, 
																											Clock slow_clock, Reset slow_reset)(Empty)
																			provisos(Max#(data_width_in, data_width_out, data_width_max),
																							 Min#(data_width_in, data_width_out, data_width_min),
																							 Mul#(data_ratio, data_width_min, data_width_max),
																							 Add#(a__, data_width_min, data_width_out),
																							 Add#(b__, data_width_in, data_width_max),
																							 Div#(data_width_in, 8, bytes_in),
																							 Div#(data_width_out, 8, bytes_out),
																							 Max#(bytes_in, bytes_out, bytes_max),
																							 Min#(bytes_in, bytes_out, bytes_min),
																							 Add#(bytes_out, bytes_in, t_bytes),
																							 Add#(bytes_max, bytes_min, t_bytes),		
																							 Mul#(c__, data_width_out, data_width_max),
																							 Add#(d__, bytes_min, bytes_in),
																							 Add#(e__, bytes_in, bytes_max),
																							 Log#(bytes_out, t_size_out),
																							 Log#(bytes_in, t_size_in));
		let v_bytes_in = valueOf(bytes_in);
		let v_bytes_out = valueOf(bytes_out);
		let v_size_out = valueOf(t_size_out);
		AXI4_Lite_Slave_Xactor_IFC #(paddr, data_width_in, userspace)  
											s_xactor <- mkAXI4_Lite_Slave_Xactor(clocked_by fast_clock, reset_by fast_reset);
		AXI4_Lite_Master_Xactor_IFC #(paddr,data_width_out,userspace) 
								 m_xactor <- mkAXI4_Lite_Master_Xactor(clocked_by slow_clock, reset_by slow_reset);
		Reg#(Bit#(4)) rd_id<-mkRegA(0, clocked_by slow_clock, reset_by slow_reset);
		Reg#(Bit#(4)) wr_id<-mkRegA(0, clocked_by slow_clock, reset_by slow_reset);
		Reg#(Bit#(8)) request_counter<-mkRegA(0,clocked_by fast_clock, reset_by fast_reset);
		Reg#(Bit#(8)) rd_response_counter<-mkRegA(0, clocked_by slow_clock, reset_by slow_reset);
    Reg#(Bit#(8)) wr_response_counter <- mkRegA(0, clocked_by slow_clock, reset_by slow_reset);
		Reg#(Bit#(8)) sync_rdburst_value <-mkSyncReg(0,fast_clock,fast_reset,slow_clock);
    Reg#(Bit#(8)) sync_wrburst_value <-mkSyncReg(0,fast_clock,fast_reset,slow_clock);
		Reg#(AXI4_Lite_Rd_Addr	#(paddr,userspace)) 
														rg_read_packet <-mkRegA(?,clocked_by fast_clock , reset_by fast_reset);
		Reg#(AXI4_Lite_Wr_Addr	#(paddr,userspace)) 
														rg_write_packet<-mkRegA(?,clocked_by fast_clock , reset_by fast_reset);
		Reg#(Tuple2#(AXI4_Lite_Wr_Addr	#(paddr,userspace), Bit#(bytes_in)))
														rg_child_metadata <-mkRegA(?,clocked_by slow_clock, reset_by slow_reset);
		Reg#(Bit#(paddr)) rg_rd_req_address <- mkRegA(0, clocked_by slow_clock, reset_by slow_reset);
		Reg#(Bit#(8)) rd_req_burst <- mkRegA(0, clocked_by slow_clock, reset_by slow_reset);
		Reg#(Bit#(8)) rd_resp_burst <- mkRegA(0, clocked_by slow_clock, reset_by slow_reset);
		Reg#(Bit#(8)) rg_burst_num <- mkRegA(0, clocked_by slow_clock, reset_by slow_reset);
		Reg#(Bool) rg_child_rd_burst <- mkRegA(False, clocked_by slow_clock, reset_by slow_reset);
		Reg#(Bool) rg_child_resp_burst <- mkRegA(False, clocked_by slow_clock, reset_by slow_reset);
		Reg#(Bool) rg_child_burst <- mkRegA(False, clocked_by slow_clock, reset_by slow_reset);
		Reg#(Bit#(data_width_in)) rg_data_resp_collect <- mkRegA(0, clocked_by slow_clock, reset_by slow_reset);
		/*=== FIFOs to synchronize data between the two clock domains ====*/
		SyncFIFOIfc#(AXI4_Lite_Rd_Addr	#(paddr,userspace))		
																			ff_rd_addr <-	mkSyncFIFO(1,fast_clock,fast_reset,slow_clock);
		SyncFIFOIfc#(AXI4_Lite_Wr_Addr	#(paddr, userspace))		
																			ff_wr_addr <-	mkSyncFIFO(1,fast_clock,fast_reset,slow_clock);
		SyncFIFOIfc#(AXI4_Lite_Wr_Data	#(data_width_in))					
																			ff_wr_data <-	mkSyncFIFO(1,fast_clock,fast_reset,slow_clock);
		SyncFIFOIfc#(AXI4_Lite_Rd_Data	#(data_width_in,userspace))	
																			ff_rd_resp <-	mkSyncFIFO(1,slow_clock,slow_reset,fast_clock);
		SyncFIFOIfc#(AXI4_Lite_Wr_Resp	#(userspace))					
																			ff_wr_resp <-	mkSyncFIFO(1,slow_clock,slow_reset,fast_clock);
		/*=================================================================*/
		mkConnection(bus1, s_xactor.axi_side);
		mkConnection(m_xactor.axi_side, bus2);
		// These rule will receive the read request from the AXI4 fabric and pass it on to the AXI4Lite fabric.
		// If the request is a burst then they are broken down to individual axi4lite read requests. These
		// are carried out in the next rule. 
		rule capture_read_requests_from_Axi4;
			let request<-pop_o(s_xactor.o_rd_addr);
			ff_rd_addr.enq(request);
			rg_read_packet<=request;
		endrule
		// In case a read-burst request is received on the fast bus, then the bursts have to broken down into
		// individual slow-bus read requests. 
		// This is rule is fired after the first read-burst request is sent to the slow_bus. This rule will continue to 
		// fire as long as the slow bus has capacity to receive a new request and the burst is not complete.
		// the difference between the each individual requests on the slow bus is only the address. All other 
		// parameters remain the same.
		//rule generate_bust_read_requests;
		//	let request=rg_read_packet;
		//	request.araddr=burst_address_generator(request.arlen, request.arsize, request.arburst,request.araddr);
		//	rg_read_packet<=request;
		//	ff_rd_addr.enq(request);
		//	if(request.arlen==request_counter)begin
		//		rd_state<=RegularReq;
		//		request_counter<=0;
		//	end
		//	else
		//		request_counter<=request_counter+1;
		//endrule
		rule send_read_request_on_slow_bus;
			let request=ff_rd_addr.first;
			if(v_bytes_in > v_bytes_out && request.arsize > fromInteger(v_bytes_out)) begin
				Bit#(8) burst_num = 1 << request.arsize;
				burst_num = burst_num >> v_size_out;
				request.arsize = fromInteger(v_bytes_out);
				rd_req_burst <= burst_num;
				rd_resp_burst <= burst_num;
				rg_child_rd_burst <= True;
				rg_child_resp_burst <= True;
			end
			else begin
				ff_rd_addr.deq;
			end
			let req_address = rg_rd_req_address;
			Bit#(paddr) addr_offset = 1 << request.arsize;
			rg_rd_req_address <= rg_rd_req_address + zeroExtend(addr_offset);
			let lite_request = AXI4_Lite_Rd_Addr {araddr: request.araddr, arsize:truncate(request.arsize),
                                            arprot: request.arprot, aruser: 0}; // arburst: 00-FIXED 01-INCR 10-WRAP
   	  m_xactor.i_rd_addr.enq(lite_request);	
		endrule
			
		rule rl_generate_burst_on_slow_bus(rg_child_rd_burst);
			let request = ff_rd_addr.first;
			let req_address = rg_rd_req_address;
			Bit#(paddr) addr_offset = 1 << request.arsize;
			//rg_rd_req_address <= burst_address_generator(request.arlen, request.arsize, request.arburst,request.araddr);
			rg_rd_req_address <= rg_rd_req_address + zeroExtend(addr_offset);
			let lite_request = AXI4_Lite_Rd_Addr {araddr: req_address, arsize:truncate(request.arsize),
                                            arprot: request.arprot, aruser: 0}; // arburst: 00-FIXED 01-INCR 10-WRAP
			if(rd_req_burst==0) begin
				ff_rd_addr.deq;
				rg_child_rd_burst <= False;
			end
			else begin
				rd_req_burst <= rd_req_burst - 1;
			end
   	  m_xactor.i_rd_addr.enq(lite_request);	
		endrule
		// This rule will capture the write request from the AXI4 fabric and pass it on to the AXI4Lite fabric.
		// In case of burst requests, they are broken down to individual requests of axi4lite writes. Care
		// needs to be taken when writes are of different sizes in settin the write-strobe correctly.
		rule capture_write_requests_from_Axi4;
			let wr_addr_req  <- pop_o (s_xactor.o_wr_addr);
	    let wr_data_req  <- pop_o (s_xactor.o_wr_data);
			ff_wr_addr.enq(wr_addr_req);
			ff_wr_data.enq(wr_data_req);
			`ifdef verbose $display($time,"\tAXIBRIDGE: Write Request"); `endif
			`ifdef verbose $display($time,"\tAddress Channel :",fshow(wr_addr_req)); `endif
			`ifdef verbose $display($time,"\tData Channel :",fshow(wr_data_req)); `endif
		endrule
		// In case a write-burst request is received on the fast bus, then the bursts have to broken down into
		// individual slow-bus write requests. 
		// This is rule is fired after the first write-burst request is sent to the slow_bus. This rule will continue to 
		// fire as long as the slow bus has capacity to receive a new request and the burst is not complete i.e.
		// fast bust xactor does not send wlast asserted.
		// The difference between the each individual requests on the slow bus is only the address. All other 
		// parameters remain the same.
		//rule generate_bust_write_requests;
		//	let request=rg_write_packet;
		//	Bit#(paddr) addr_offset = 1 << request.arsize;
		//	rg_rd_req_address <= rg_rd_req_address + zeroExtend(addr_offset);
		//	//request.awaddr=burst_address_generator(request.awlen, request.awsize, request.awburst,request.awaddr);
	  //    let wr_data_req  <- pop_o (s_xactor.o_wr_data);
		//	ff_wr_addr.enq(request);
		//	ff_wr_data.enq(wr_data_req);
		//	rg_write_packet<=request;
		//	if(wr_data_req.wlast)begin
		//		wr_state<=RegularReq;
		//	end
		//	`ifdef verbose $display($time,"\tAXIBRIDGE: Burst Write Request"); `endif
		//	`ifdef verbose $display($time,"\tAddress Channel :",fshow(rg_write_packet)); `endif
		//	`ifdef verbose $display($time,"\tData Channel :",fshow(wr_data_req)); `endif
		//endrule
		rule send_write_request_on_slow_bus;
			let wr_addr_req  = ff_wr_addr.first;
	    let wr_data_req  = ff_wr_data.first;
			let lv_awsize = wr_addr_req.awsize;
			if(valueOf(bytes_in) > valueOf(bytes_out) && 
												wr_addr_req.awsize>fromInteger(v_size_out)) begin
				lv_awsize = fromInteger(v_size_out);
				rg_child_burst <= True;
				Bit#(8) burst_num = 1 << lv_awsize;
				burst_num = burst_num >> v_size_out;
				rg_burst_num <= burst_num;
				sync_wrburst_value <= burst_num;
			end
			else begin
				ff_wr_data.deq;
				ff_wr_addr.deq;
			end
			Tuple3#(Bit#(bytes_out), Bit#(bytes_in), Bit#(data_width_out)) strb_mod;
			strb_mod = strb_out(wr_data_req.wstrb, wr_data_req.wdata);	
			Bit#(bytes_out) lv_strb_out = tpl_1(strb_mod);
			Bit#(bytes_in) lv_strb_in = tpl_2(strb_mod);
			Bit#(data_width_out) lv_data = tpl_3(strb_mod);
			Bit#(paddr) addr_offset = 1 << wr_addr_req.awsize;
			wr_addr_req.awaddr = wr_addr_req.awaddr + addr_offset;
			rg_child_metadata <= tuple2(wr_addr_req, wr_data_req.wstrb); 
			let aw = AXI4_Lite_Wr_Addr {awaddr: wr_addr_req.awaddr, awuser:0, 
                                  awprot: wr_addr_req.awprot, awsize: truncate(wr_addr_req.awsize)}; // arburst: 00-FIXED 01-INCR 10-WRAP
			let w  = AXI4_Lite_Wr_Data {wdata:  lv_data, wstrb: lv_strb_out};
			m_xactor.i_wr_addr.enq(aw);
			m_xactor.i_wr_data.enq(w);
		endrule
		rule send_child_burst_data(rg_child_burst);
			let {wr_addr_req, lv_strb} = rg_child_metadata;
	    let wr_data_req  = ff_wr_data.first;
			Tuple3#(Bit#(bytes_out), Bit#(bytes_in), Bit#(data_width_out)) strb_mod;
			strb_mod = strb_out(lv_strb, wr_data_req.wdata);	
			Bit#(bytes_out) lv_strb_out = tpl_1(strb_mod);
			Bit#(bytes_in) lv_strb_in = tpl_2(strb_mod);
			Bit#(data_width_out) lv_data = tpl_3(strb_mod);
			Bit#(paddr) addr_offset = 1 << wr_addr_req.awsize;
			wr_addr_req.awaddr = wr_addr_req.awaddr + addr_offset;
			if(rg_burst_num !=0) begin
				rg_burst_num <= rg_burst_num -1;
			end
			else begin
				rg_child_burst <= False;
			end
			rg_child_metadata <= tuple2(wr_addr_req, lv_strb);
			let aw = AXI4_Lite_Wr_Addr {awaddr: wr_addr_req.awaddr, awuser:0, 
                                  awprot: wr_addr_req.awprot, awsize: truncate(wr_addr_req.awsize)}; // arburst: 00-FIXED 01-INCR 10-WRAP
			let w  = AXI4_Lite_Wr_Data {wdata: lv_data, wstrb: lv_strb_out};
			m_xactor.i_wr_addr.enq(aw);
			m_xactor.i_wr_data.enq(w);
		endrule
		// This rule forwards the read response from the AXI4Lite to the AXI4 fabric.
		rule capture_read_responses;
			let response <- pop_o (m_xactor.o_rd_data);
			Bit#(data_width_min) resp_collect_min = truncate(response.rdata);
			Bit#(data_width_max) resp_collect_max = duplicate(response.rdata);
			Bit#(data_width_in) resp_collect = truncate(resp_collect_max);
			if(rg_child_resp_burst) begin
				resp_collect = read_data_resp(response.rdata, rg_data_resp_collect);  
				rg_data_resp_collect <= resp_collect;
				if(rd_resp_burst==0) begin
					AXI4_Lite_Rd_Data#(data_width_in,userspace) r = AXI4_Lite_Rd_Data {rresp: response.rresp, 
																																	 rdata: resp_collect, 
																																	 ruser: 0};
					if(rd_resp_burst==0) begin
						rd_resp_burst <= rd_resp_burst - 1;
					end
					else begin
						rg_child_resp_burst <= False;
					end
					ff_rd_resp.enq(r);
				end
			end
			else begin
					AXI4_Lite_Rd_Data#(data_width_in,userspace) r = AXI4_Lite_Rd_Data {rresp: response.rresp, 
																																	 rdata: resp_collect,
																																	 ruser: 0};
					if(rd_response_counter==sync_rdburst_value)
						rd_response_counter<=0;
					else
						rd_response_counter<=rd_response_counter+1;
					ff_rd_resp.enq(r);
			end
		endrule
		rule send_read_response_on_fast_bus;
			ff_rd_resp.deq;
			s_xactor.i_rd_data.enq(ff_rd_resp.first);
		endrule
		rule capture_write_responses;
			let response<-pop_o(m_xactor.o_wr_resp);
			let b = AXI4_Lite_Wr_Resp {bresp: response.bresp, buser:0};
      if(wr_response_counter == sync_wrburst_value) begin
			  ff_wr_resp.enq(b);
        wr_response_counter <= 0;
      end
      else
        wr_response_counter <= wr_response_counter + 1;
		endrule
		rule send_write_response_on_fast_bus;
			ff_wr_resp.deq;
			s_xactor.i_wr_resp.enq(ff_wr_resp.first);
		endrule
		//interface axi_slave=s_xactor.axi_side;
		//interface axi4_lite_master=m_xactor.axi_side;
	endmodule	
endinstance
endpackage
