
	/*=== Project imports ====*/
	import AXI4_Fabric::*;
	import AXI4_Types ::*;
	import Semi_FIFOF	::*;
	import FIFO ::*;
	import axi_addr_generator::*;
	import bridge_types ::*;
	import bridge_functions ::*;
	import Connectable ::*;
	/*======================*/
	/*=== Package imports ===*/
	import Clocks::*;
	import BUtils ::*;
	/*=======================*/

`define VERBOSITY 2
instance ConnectableClocks#(AXI4_Master_IFC#(paddr,data_width_in,userspace),   
																				AXI4_Slave_IFC#(paddr,data_width_out,userspace))
																		provisos(Max#(data_width_in, data_width_out, data_width_max),
																						 Min#(data_width_in, data_width_out, data_width_min),
																						 Mul#(data_ratio, data_width_min, data_width_max));
	module mkConnectionClocks#(AXI4_Master_IFC#(paddr,data_width_in,userspace) bus1, 
																															Clock fast_clock, Reset fast_reset, 
																AXI4_Slave_IFC#(paddr,data_width_out,userspace) bus2, 
																											Clock slow_clock, Reset slow_reset)(Empty)
																			provisos(Max#(data_width_in, data_width_out, data_width_max),
																							 Min#(data_width_in, data_width_out, data_width_min),
																							 Mul#(data_ratio, data_width_min, data_width_max),
																							 Div#(data_width_in, 8, bytes_in),
																							 Div#(data_width_out, 8, bytes_out),
																							 Max#(bytes_in, bytes_out, bytes_max),
																							 Min#(bytes_in, bytes_out, bytes_min),
																							 Add#(bytes_out, bytes_in, t_bytes),
																							 Add#(bytes_max, bytes_min, t_bytes),		
																							 Add#(a__, bytes_min, bytes_in),
																							 Add#(b__, bytes_in, bytes_max),
																							 Log#(bytes_out, t_size_out),
																							 Log#(bytes_in, t_size_in));
		let v_size_out = valueOf(t_size_out);
		let v_size_in = valueOf(t_size_in);

		AXI4_Slave_Xactor_IFC #(paddr, data_width_in, userspace)  
											s_xactor <- mkAXI4_Slave_Xactor(clocked_by fast_clock, reset_by fast_reset);
		AXI4_Master_Xactor_IFC #(paddr,data_width_out,userspace) 
								 m_xactor <- mkAXI4_Master_Xactor(clocked_by slow_clock, reset_by slow_reset);

		Reg#(BridgeState) rd_state <-mkRegA(RegularReq,clocked_by fast_clock, reset_by fast_reset);
		Reg#(BridgeState) wr_state <-mkRegA(RegularReq,clocked_by fast_clock, reset_by fast_reset);
		Reg#(Bit#(4)) rd_id<-mkRegA(0, clocked_by slow_clock, reset_by slow_reset);
		Reg#(Bit#(4)) wr_id<-mkRegA(0, clocked_by slow_clock, reset_by slow_reset);
		Reg#(Bit#(8)) request_counter<-mkRegA(0,clocked_by fast_clock, reset_by fast_reset);
		Reg#(Bit#(8)) rd_response_counter<-mkRegA(0, clocked_by slow_clock, reset_by slow_reset);
    Reg#(Bit#(8)) wr_response_counter <- mkRegA(0, clocked_by slow_clock, reset_by slow_reset);
		Reg#(Bit#(8)) sync_rdburst_value <-mkSyncReg(0,fast_clock,fast_reset,slow_clock);
    //Reg#(Bit#(8)) sync_wrburst_value <-mkSyncReg(0,fast_clock,fast_reset,slow_clock);
		Reg#(AXI4_Rd_Addr	#(paddr,userspace)) 
														rg_read_packet <-mkRegA(?,clocked_by fast_clock , reset_by fast_reset);
		Reg#(AXI4_Wr_Addr	#(paddr,userspace)) 
														rg_write_packet<-mkRegA(?,clocked_by slow_clock , reset_by slow_reset);
		Reg#(Bool) rg_wr_child_burst <- mkRegA(False, clocked_by slow_clock, reset_by slow_reset);
		Reg#(Bool) rg_wr_data_burst <- mkRegA(False, clocked_by slow_clock, reset_by slow_reset);
		Reg#(Bool) rg_linear_burst <- mkRegA(False, clocked_by slow_clock, reset_by slow_reset);
		Reg#(Bit#(8)) rg_tier2_burst <- mkRegA(0, clocked_by slow_clock , reset_by slow_reset);
		Reg#(Bit#(8)) rg_parent_burst_awlen <- mkRegA(0, clocked_by slow_clock, reset_by slow_reset);
		Reg#(Bit#(8)) rg_m_parent_burst_awlen <- mkRegA(0, clocked_by fast_clock, reset_by fast_reset);
		Reg#(Tuple2#(Bit#(8), Bit#(3)))	rg_rd_burst_metadata 
																 <- mkRegA(tuple2(0,0), clocked_by fast_clock , reset_by fast_reset);
		Reg#(Bool) rg_sync_rd_child_burst <- mkSyncReg(False,fast_clock,fast_reset,slow_clock);
		Reg#(Bit#(data_width_in)) rg_data_resp_collect 
																			<- mkRegA(0, clocked_by slow_clock , reset_by slow_reset);
		Reg#(Bit#(8)) rg_child_awlen <- mkRegA(0, clocked_by slow_clock, reset_by slow_reset);
		Reg#(Bit#(8)) rg_parent_awlen <- mkRegA(0, clocked_by slow_clock, reset_by slow_reset);
		Reg#(AXI4_Wr_Data	#(data_width_in)) rg_data_packet 
																	 <-mkRegA(?, clocked_by slow_clock, reset_by slow_reset); 

		/*=== FIFOs to synchronize data between the two clock domains ====*/
		SyncFIFOIfc#(AXI4_Rd_Addr	#(paddr,userspace))		
																			ff_rd_addr <-	mkSyncFIFO(1,fast_clock,fast_reset,slow_clock);
		SyncFIFOIfc#(AXI4_Wr_Addr	#(paddr, userspace))		
																			ff_wr_addr <-	mkSyncFIFO(1,fast_clock,fast_reset,slow_clock);
		SyncFIFOIfc#(AXI4_Wr_Data	#(data_width_in))					
																			ff_wr_data <-	mkSyncFIFO(1,fast_clock,fast_reset,slow_clock);

		SyncFIFOIfc#(AXI4_Rd_Data	#(data_width_in,userspace))	
																			ff_rd_resp <-	mkSyncFIFO(1,slow_clock,slow_reset,fast_clock);
		SyncFIFOIfc#(AXI4_Wr_Resp	#(userspace))					
																			ff_wr_resp <-	mkSyncFIFO(1,slow_clock,slow_reset,fast_clock);
		/*=================================================================*/

		//function Tuple3#(Bit#(bytes_out), Bit#(bytes_in), Bit#(data_width_out)) 
		//																		strb_out(Bit#(bytes_in) in_burst, Bit#(data_width_in) data)
		//																		provisos(Add#(data_width_out, data_width_in, t_data_width));
		//	let v_bytes_in = valueOf(bytes_in);
		//	let v_bytes_out = valueOf(bytes_out);
		//	let v_data_out = valueOf(data_width_out);
		//	let v_bytes_max = valueOf(bytes_max);
		//	let v_bytes_min = valueOf(bytes_min);
		//	Bit#(t_data_width) lv_data_out = zeroExtend(data);
		//	Bit#(t_bytes) burst_out;
		//	if(valueOf(data_width_out) >= valueOf(data_width_in)) begin
		//		lv_data_out = zeroExtend(data);
		//		if(valueOf(bytes_out) >= valueOf(bytes_in)) 
		//			burst_out = zeroExtend(in_burst);
		//	end
		//	else begin 
		//		Bit#(bytes_max) burst_bytes = zeroExtend(in_burst);
		//		Bit#(bytes_min) burst_min = truncate(burst_out);
		//		for(Integer i=0; i<valueOf(data_ratio); i=i+1) begin
		//			burst_bytes =	in_burst[(i+1)*v_bytes_min-1:i*v_bytes_min];
		//			if(burst_bytes!=0) begin
		//				burst_out = zeroExtend(burst_bytes);
		//				Bit#(bytes_min) lv_zeroes= 0;
		//				in_burst[(i+1)*v_bytes_min-1:i*v_bytes_min] = lv_zeroes;
		//				lv_data_out = data[(i+1)*v_data_out-1:i*v_bytes_out];	
		//			end
		//		end
		//	end
		//	return tuple3(truncate(burst_out), in_burst, truncate(lv_data_out));
		//endfunction
			
			
		//function Bit#(8) awlen_out(Bit#(8) awlen_in, Bit#(3) awsize);
		//	Bit#(8) awlen_out;
		//	Bit#(3) awsize_out = zeroExtend(fromInteger(data_width_out));
		//	if(awsize <= awsize_out)
		//		awlen_out = awlen_in;
		//	else begin
		//		awlen_out = zeroExtend(awsize/awsize_out);
		//	end
		//endfunction

		mkConnection(bus1, s_xactor.axi_side);
		mkConnection(m_xactor.axi_side, bus2);

		// These rule will receive the read request from the AXI4 fabric and pass it on to the AXI4Lite fabric.
		// If the request is a burst then they are broken down to individual axi4lite read requests. These
		// are carried out in the next rule. 
		rule capture_read_requests_from_Axi4(rd_state==RegularReq);
			let request<-pop_o(s_xactor.o_rd_addr);
			Bit#(3) child_bus_arsize = request.arsize;
			rg_m_parent_burst_awlen <= request.arlen-1;
			rg_read_packet<=request;
			sync_rdburst_value<=request.arlen;
			if(valueOf(bytes_in) > valueOf(bytes_out) && request.arsize>fromInteger(v_size_out)) begin
					Bit#(8) lv_awlen = 1;
					lv_awlen = lv_awlen << request.arsize;
					lv_awlen = lv_awlen >> fromInteger(v_size_out);
					lv_awlen = lv_awlen-1;
					rd_state <= BurstReq;
					request.arlen=lv_awlen;	
					request.arsize=fromInteger(v_size_out);
					rg_rd_burst_metadata <= tuple2(lv_awlen, fromInteger(v_size_out));
					rg_sync_rd_child_burst <= True;
			end
			ff_rd_addr.enq(request);
			if(`VERBOSITY>=1) $display($time,"\tAXI parent: Read Request");
			if(`VERBOSITY>=1) $display($time,"\tRead Channel :",fshow(request));
			//if(request.arlen!=0) begin
			//	rd_state<=BurstReq;
			//end
		endrule
		// In case a read-burst request is received on the fast bus, then the bursts have to broken down into
		// individual slow-bus read requests. 
		// This is rule is fired after the first read-burst request is sent to the slow_bus. This rule will continue to 
		// fire as long as the slow bus has capacity to receive a new request and the burst is not complete.
		// the difference between the each individual requests on the slow bus is only the address. All other 
		// parameters remain the same.
		rule generate_bust_read_requests(rd_state==BurstReq);
			let request=rg_read_packet;
			request.araddr=burst_address_generator(request.arlen, request.arsize, request.arburst,request.araddr);
			rg_read_packet<=request;
			request.arlen = tpl_1(rg_rd_burst_metadata);
			request.arsize = tpl_2(rg_rd_burst_metadata);
			ff_rd_addr.enq(request);
			if(rg_m_parent_burst_awlen==0)
				rd_state<=RegularReq;
			else begin
				rg_m_parent_burst_awlen <= rg_m_parent_burst_awlen - 1;
			end
			//if(request.arlen==request_counter)begin
			//	request_counter<=0;
			//	if(rg_m_parent_burst_awlen==0) begin
			//		rd_state<=RegularReq;
			//	end
			//	else begin
			//		rg_m_parent_burst_awlen <= rg_m_parent_burst_awlen - 1;
			//	end
			//end
			//else
			//	request_counter<=request_counter+1;
			if(`VERBOSITY>=1) $display($time,"\tRead Channel :",fshow(request));
		endrule

		rule send_read_request_on_slow_bus;
			let request=ff_rd_addr.first;
			ff_rd_addr.deq;
		 	request = AXI4_Rd_Addr {araddr: request.araddr, 
																	aruser: request.aruser,
																	arsize: request.arsize,
																	arlen : request.arlen,
																	arburst : request.arburst,
																	arid : request.arid }; 
																	// arburst: 00-FIXED 01-INCR 10-WRAP
   	  m_xactor.i_rd_addr.enq(request);	
			rd_id<=request.arid;
			if(`VERBOSITY>=1) $display($time,"\tAXI parent: Read Request");
			if(`VERBOSITY>=1) $display($time,"\tRead Channel :",fshow(request));
		endrule
		// This rule will capture the write request from the AXI4 fabric and pass it on to the AXI4Lite fabric.
		// In case of burst requests, they are broken down to individual requests of axi4lite writes. Care
		// needs to be taken when writes are of different sizes in settin the write-strobe correctly.
		rule capture_write_requests_from_Axi4(wr_state==RegularReq);
			let wr_addr_req  <- pop_o (s_xactor.o_wr_addr);
	      let wr_data_req  <- pop_o (s_xactor.o_wr_data);
			ff_wr_addr.enq(wr_addr_req);
			ff_wr_data.enq(wr_data_req);
			//rg_write_packet<=wr_addr_req;
      //sync_wrburst_value <= wr_addr_req.awlen;
			if(wr_addr_req.awlen!=0) begin
				wr_state<=BurstReq;
			end
			if(`VERBOSITY>=2) $display($time,"\tAXI parent: Write Request");
			if(`VERBOSITY>=2) $display($time,"\tWrite Address Channel :",fshow(wr_addr_req));
			if(`VERBOSITY>=2) $display($time,"\tWrite Data Channel :",fshow(wr_data_req)); 
		endrule
		// In case a write-burst request is received on the fast bus, then the bursts have to broken down into
		// individual slow-bus write requests. 
		// This is rule is fired after the first write-burst request is sent to the slow_bus. This rule will continue to 
		// fire as long as the slow bus has capacity to receive a new request and the burst is not complete i.e.
		// fast bust xactor does not send wlast asserted.
		// The difference between the each individual requests on the slow bus is only the address. All other 
		// parameters remain the same.
		rule generate_bust_write_requests(wr_state==BurstReq);
			//let request=rg_write_packet;
			//request.awaddr=burst_address_generator(request.awlen, request.awsize, request.awburst,request.awaddr);
	      let wr_data_req  <- pop_o (s_xactor.o_wr_data);
			//ff_wr_addr.enq(request);
			ff_wr_data.enq(wr_data_req);
			if(`VERBOSITY>=2) $display($time,"\tAXIBRIDGE: Burst Write Request");
			if(`VERBOSITY>=2) $display($time,"\tWrite Data Channel :",fshow(wr_data_req)); 
			if(wr_data_req.wlast)begin
				wr_state<=RegularReq;
				if(`VERBOSITY>=2) $display($time,"\tWrite Data Last :",fshow(wr_data_req)); 
			end
		endrule
		
		rule send_write_request_on_child_bus(!rg_wr_data_burst && !rg_linear_burst);
			let wr_addr_req  = ff_wr_addr.first;
	    let wr_data_req  = ff_wr_data.first;
			Bit#(8) lv_awlen= wr_addr_req.awlen;
			let lv_awsize = wr_addr_req.awsize;
			if(valueOf(bytes_in) > valueOf(bytes_out) && wr_addr_req.awsize>fromInteger(v_size_out)) begin
					lv_awlen = 1;
					lv_awlen = lv_awlen << wr_addr_req.awsize;
					lv_awlen = lv_awlen >> fromInteger(v_size_out);
					lv_awlen = lv_awlen-1;
					rg_tier2_burst <= lv_awlen; //TODO this depends on the number of requests in flight
					rg_wr_child_burst <= True;
					rg_wr_data_burst <= True;	
					rg_parent_awlen <= wr_addr_req.awlen;
			end
			else if(lv_awlen!=0) begin
				rg_linear_burst <= True;
				ff_wr_addr.deq;
			end
			else begin
				ff_wr_addr.deq;
			end
			ff_wr_data.deq;
			rg_child_awlen <= lv_awlen - 1;
			
			if(wr_addr_req.awsize > fromInteger(v_size_out))
				lv_awsize = fromInteger(v_size_out);
			Tuple3#(Bit#(bytes_out), Bit#(bytes_in), Bit#(data_width_out)) strb_mod;
			strb_mod = strb_out(wr_data_req.wstrb, wr_data_req.wdata);
			Bit#(bytes_out) lv_strb_out = tpl_1(strb_mod);
			Bit#(bytes_in) lv_strb_in = tpl_2(strb_mod);
			Bit#(data_width_out) lv_data = tpl_3(strb_mod);
			
			//let lv_awlen_out = awlen_out(wr_addr_req.awlen, wr_addr_req.awsize);
			AXI4_Wr_Addr#(paddr, userspace) aw = AXI4_Wr_Addr {awaddr: wr_addr_req.awaddr, 
														 awuser:0, 
														 awlen : lv_awlen,
														 awsize: lv_awsize,
														 awburst : wr_addr_req.awburst,
														 awid : wr_addr_req.awid}; // arburst: 00-FIXED 01-INCR 10-WRAP

			AXI4_Wr_Data#(data_width_out) w  = AXI4_Wr_Data {wdata:  lv_data, 
														 												wstrb: lv_strb_out,
														 												wid : wr_data_req.wid,
														 												wlast : lv_awlen==0};

			wr_addr_req.awaddr=burst_address_generator(wr_addr_req.awlen, wr_addr_req.awsize, 
																													wr_addr_req.awburst,wr_addr_req.awaddr);
			wr_addr_req.awlen = lv_awlen;
			wr_addr_req.awsize = lv_awsize;
			rg_write_packet<=wr_addr_req;

			wr_data_req.wstrb = lv_strb_in;
			rg_data_packet <= wr_data_req;
			m_xactor.i_wr_addr.enq(aw);
			m_xactor.i_wr_data.enq(w);
			wr_id <= wr_addr_req.awid;
			if(`VERBOSITY>=2) $display($time,"\tAXI child: Write Request");
			if(`VERBOSITY>=2) $display($time,"\tWrite Address Channel rule 1:",fshow(aw));
			if(`VERBOSITY>=2) $display($time,"\tWrite Data Channel :",fshow(w)); 
		endrule

		rule rl_initiate_burst_child(!rg_wr_child_burst && rg_wr_data_burst);
			let wr_addr_req = ff_wr_addr.first;
			let wr_data_req = ff_wr_data.first;
			let request=rg_write_packet;
			
			Tuple3#(Bit#(bytes_out), Bit#(bytes_in), Bit#(data_width_out)) strb_mod;
			strb_mod = strb_out(wr_data_req.wstrb, wr_data_req.wdata);
			Bit#(bytes_out) lv_strb_out = tpl_1(strb_mod);
			Bit#(bytes_in) lv_strb_in = tpl_2(strb_mod);
			Bit#(data_width_out) lv_data = tpl_3(strb_mod);
			//let lv_awlen_out = awlen_out(wr_addr_req.awlen, wr_addr_req.awsize);
			let aw = AXI4_Wr_Addr {awaddr: request.awaddr, 
														 awuser:0, 
														 awlen : request.awlen,
														 awsize: request.awsize,
														 awburst : wr_addr_req.awburst,
														 awid : wr_addr_req.awid}; // arburst: 00-FIXED 01-INCR 10-WRAP
			let w  = AXI4_Wr_Data {wdata:  lv_data, 
														 wstrb: lv_strb_out,
														 wid : wr_data_req.wid,
														 wlast : False};
			request.awaddr=burst_address_generator(request.awlen, request.awsize, 
																														request.awburst,request.awaddr);
			rg_write_packet<=request;
			wr_data_req.wstrb = lv_strb_in;
			rg_data_packet <= wr_data_req;
			rg_child_awlen <= request.awlen;
			if(rg_parent_awlen!=0)
				rg_parent_awlen <= rg_parent_awlen - 1;
			else
				ff_wr_addr.deq;
			ff_wr_data.deq;
			if(`VERBOSITY>=2) $display($time,"\tWrite Data Channel Child burst 0 rule 2:",fshow(w)); 
			m_xactor.i_wr_addr.enq(aw);
			m_xactor.i_wr_data.enq(w);
		endrule

		rule rl_burst_child(rg_wr_child_burst && rg_wr_data_burst);
			let wr_data_req = ff_wr_data.first;
			let data_request = rg_data_packet;
			Tuple3#(Bit#(bytes_out), Bit#(bytes_in), Bit#(data_width_out)) strb_mod;
			strb_mod = strb_out(rg_data_packet.wstrb, rg_data_packet.wdata);
			Bit#(bytes_out) lv_strb_out = tpl_1(strb_mod);
			Bit#(bytes_in) lv_strb_in = tpl_2(strb_mod);
			Bit#(data_width_out) lv_data = tpl_3(strb_mod);
			
			data_request.wstrb = lv_strb_in;
			rg_data_packet <= data_request;
			
			Bool lv_wlast = False; 
			if(rg_child_awlen!=0)
				rg_child_awlen <= rg_child_awlen - 1;
			else begin
				ff_wr_data.deq;
				rg_wr_child_burst <= False;
				if(rg_parent_awlen==0)
					rg_wr_data_burst <= False;
			end

			let w  = AXI4_Wr_Data {wdata:  lv_data, 
														 wstrb: lv_strb_out,
														 wid : wr_data_req.wid,
														 wlast : lv_wlast};
			if(`VERBOSITY>=2) $display($time,"\tWrite Data Channel Child burst awlen %h rule 3:",rg_child_awlen,fshow(w)); 
			//wr_data_req.astrb = lv_strb_in;
			//rg_data_request <= wr_data_req;
			m_xactor.i_wr_data.enq(w);
		endrule

		rule rl_burst_linear(rg_linear_burst);
			let wr_data_req = ff_wr_data.first;
			//let {lv_strb_out, lv_strb_in, lv_data} = strb_out(rg_data_packet.wstrb,
			//																										 rg_data_packet.wdata);	
			//data_request.wstrb = lv_strb_in;
			//rg_data_packet <= data_request;
			Bit#(TAdd#(data_width_out, data_width_in)) lv_data = zeroExtend(wr_data_req.wdata);	
			Bit#(TAdd#(TDiv#(data_width_out,8), TDiv#(data_width_in,8))) 
																													lv_strb = zeroExtend(wr_data_req.wstrb);
			if(rg_child_awlen!=0)
				rg_child_awlen <= rg_child_awlen - 1;
			else begin
				rg_linear_burst <= False;
			end
			ff_wr_data.deq;
			if(`VERBOSITY>=2) $display($time,"\tData Channel :",fshow(wr_data_req)); 

			AXI4_Wr_Data#(data_width_out) w  = AXI4_Wr_Data {wdata:  truncate(lv_data), 
														 wstrb: truncate(lv_strb),
														 wid : wr_data_req.wid,
														 wlast : wr_data_req.wlast};
			//wr_data_req.astrb = lv_strb_in;
			//rg_data_request <= wr_data_req;
			m_xactor.i_wr_data.enq(w);
		endrule

		// This rule forwards the read response from the AXI4 to the AXI4 fabric.
		rule capture_read_responses;
			let response <- pop_o (m_xactor.o_rd_data);
			//AXI4_Resp rresp= case(response.rresp)
			//	AXI4_LITE_OKAY  : AXI4_OKAY;
			//	AXI4_LITE_EXOKAY: AXI4_EXOKAY;
			//	AXI4_LITE_SLVERR: AXI4_SLVERR;
			//	AXI4_LITE_DECERR: AXI4_DECERR;
			//	default: AXI4_SLVERR; endcase;
			Bit#(TAdd#(data_width_out, data_width_in)) temp_resp = zeroExtend(response.rdata);//TODO need to dulicate this stuff
			if(rg_sync_rd_child_burst) begin
				Bit#(data_width_in) resp_collect = read_data_resp(response.rdata, rg_data_resp_collect);  
				//Bit#(TLog#(data_width_in)) data_in_bytes = (1<<arsize_req)*8;
				//response.rdata = resp_collect[data_in_bytes-1:0];
				rg_data_resp_collect <= resp_collect;
				AXI4_Rd_Data#(data_width_in,userspace) r = AXI4_Rd_Data {rresp: response.rresp, 
																																	rdata: resp_collect,
																																	rlast:rd_response_counter==sync_rdburst_value, 
																																	ruser: 0, rid:rd_id};
				if(response.rlast) begin
					ff_rd_resp.enq(r);
					if(rd_response_counter==sync_rdburst_value)
						rd_response_counter<=0;
					else begin
						rd_response_counter<=rd_response_counter+1;
					end
				end
			end
			else begin
				AXI4_Rd_Data#(data_width_in,userspace) r = AXI4_Rd_Data {rresp: response.rresp, 
																																	rdata: truncate(temp_resp),
																																	rlast:rd_response_counter==sync_rdburst_value, 
																																	ruser: 0, rid:rd_id};
				if(rd_response_counter==sync_rdburst_value)
					rd_response_counter<=0;
				else begin
					rd_response_counter<=rd_response_counter+1;
				end
				ff_rd_resp.enq(r);
			end
		endrule
		rule send_read_response_on_fast_bus;
			ff_rd_resp.deq;
			s_xactor.i_rd_data.enq(ff_rd_resp.first);
		endrule
		rule capture_write_responses;
			let response<-pop_o(m_xactor.o_wr_resp);
			//AXI4_Resp bresp= case(response.bresp)
			//	AXI4_LITE_OKAY  : AXI4_OKAY;
			//	AXI4_LITE_EXOKAY: AXI4_EXOKAY;
			//	AXI4_LITE_SLVERR: AXI4_SLVERR;
			//	AXI4_LITE_DECERR: AXI4_DECERR;
			//	default: AXI4_SLVERR; endcase;
			let b = AXI4_Wr_Resp {bresp: response.bresp, buser:0, bid:wr_id};
      if(wr_response_counter == rg_tier2_burst) begin
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

//instance Connectable#(AXI4_Master_IFC#(paddr,data_width_in,userspace),   
//																				AXI4_Slave_IFC#(paddr,data_width_out,userspace))
//																		provisos(Max#(data_width_in, data_width_out, data_width_max),
//																						 Min#(data_width_in, data_width_out, data_width_min),
//																						 Mul#(data_ratio, data_width_min, data_width_max));
//	module mkConnection#(AXI4_Master_IFC#(paddr,data_width_in,userspace) bus1,
//																AXI4_Slave_IFC#(paddr,data_width_out,userspace) bus2) (Empty)
//																			provisos(Max#(data_width_in, data_width_out, data_width_max),
//																							 Min#(data_width_in, data_width_out, data_width_min),
//																							 Mul#(data_ratio, data_width_min, data_width_max),
//																							 Div#(data_width_in, 8, bytes_in),
//																							 Div#(data_width_out, 8, bytes_out),
//																							 Max#(bytes_in, bytes_out, bytes_max),
//																							 Min#(bytes_in, bytes_out, bytes_min),
//																							 Add#(bytes_out, bytes_in, t_bytes),
//																							 Add#(bytes_max, bytes_min, t_bytes),		
//																							 Add#(a__, bytes_min, bytes_in),
//																							 Add#(b__, bytes_in, bytes_max),
//																							 Log#(bytes_out, t_size_out),
//																							 Log#(bytes_in, t_size_in));
//		let v_size_out = valueOf(t_size_out);
//		let v_size_in = valueOf(t_size_in);
//
//		AXI4_Slave_Xactor_IFC #(paddr, data_width_in, userspace)  
//											s_xactor <- mkAXI4_Slave_Xactor();
//		AXI4_Master_Xactor_IFC #(paddr,data_width_out,userspace) 
//								 m_xactor <- mkAXI4_Master_Xactor();
//
//		Reg#(BridgeState) rd_state <-mkRegA(RegularReq);
//		Reg#(BridgeState) wr_state <-mkRegA(RegularReq);
//		Reg#(Bit#(4)) rd_id<-mkRegA(0);
//		Reg#(Bit#(4)) wr_id<-mkRegA(0);
//		Reg#(Bit#(8)) request_counter<-mkRegA(0);
//		Reg#(Bit#(8)) rd_response_counter<-mkRegA(0);
//    Reg#(Bit#(8)) wr_response_counter <- mkRegA(0);
//		Reg#(Bit#(8)) sync_rdburst_value <-mkRegA(0);
//    //Reg#(Bit#(8)) sync_wrburst_value <-mkSyncReg(0,fast_clock,fast_reset,slow_clock);
//		Reg#(AXI4_Rd_Addr	#(paddr,userspace)) 
//														rg_read_packet <-mkRegA(?);
//		Reg#(AXI4_Wr_Addr	#(paddr,userspace)) 
//														rg_write_packet<-mkRegA(?);
//		Reg#(Bool) rg_wr_child_burst <- mkRegA(False);
//		Reg#(Bool) rg_wr_data_burst <- mkRegA(False);
//		Reg#(Bool) rg_linear_burst <- mkRegA(False);
//		Reg#(Bit#(8)) rg_parent_burst_awlen <- mkRegA(0);
//		Reg#(Bit#(8)) rg_tier2_burst <- mkRegA(0);
//		Reg#(Tuple2#(Bit#(8), Bit#(3)))	rg_rd_burst_metadata <- mkRegA(tuple2(0,0));
//		Reg#(Bool) rg_sync_rd_child_burst <- mkRegA(False);
//		Reg#(Bit#(data_width_in)) rg_data_resp_collect	<- mkRegA(0);
//		Reg#(Bit#(8)) rg_child_awlen <- mkRegA(0);
//		Reg#(Bit#(8)) rg_parent_awlen <- mkRegA(0);
//		Reg#(AXI4_Wr_Data	#(data_width_in)) rg_data_packet <-mkRegA(?); 
//
//		/*=== FIFOs to synchronize data between the two clock domains ====*/
//		FIFO#(AXI4_Rd_Addr	#(paddr,userspace))		ff_rd_addr <-	mkFIFO();
//		FIFO#(AXI4_Wr_Addr	#(paddr, userspace))	ff_wr_addr <-	mkFIFO();
//		FIFO#(AXI4_Wr_Data	#(data_width_in))	ff_wr_data <-	mkFIFO();
//
//		FIFO#(AXI4_Rd_Data	#(data_width_in,userspace))	ff_rd_resp <-	mkFIFO();
//		FIFO#(AXI4_Wr_Resp	#(userspace))	ff_wr_resp <-	mkFIFO();
//		/*=================================================================*/
//
//		//function Tuple3#(Bit#(bytes_out), Bit#(bytes_in), Bit#(data_width_out)) 
//		//																		strb_out(Bit#(bytes_in) in_burst, Bit#(data_width_in) data)
//		//																		provisos(Add#(data_width_out, data_width_in, t_data_width));
//		//	let v_bytes_in = valueOf(bytes_in);
//		//	let v_bytes_out = valueOf(bytes_out);
//		//	let v_data_out = valueOf(data_width_out);
//		//	let v_bytes_max = valueOf(bytes_max);
//		//	let v_bytes_min = valueOf(bytes_min);
//		//	Bit#(t_data_width) lv_data_out = zeroExtend(data);
//		//	Bit#(t_bytes) burst_out;
//		//	if(valueOf(data_width_out) >= valueOf(data_width_in)) begin
//		//		lv_data_out = zeroExtend(data);
//		//		if(valueOf(bytes_out) >= valueOf(bytes_in)) 
//		//			burst_out = zeroExtend(in_burst);
//		//	end
//		//	else begin 
//		//		Bit#(bytes_max) burst_bytes = zeroExtend(in_burst);
//		//		Bit#(bytes_min) burst_min = truncate(burst_out);
//		//		for(Integer i=0; i<valueOf(data_ratio); i=i+1) begin
//		//			burst_bytes =	in_burst[(i+1)*v_bytes_min-1:i*v_bytes_min];
//		//			if(burst_bytes!=0) begin
//		//				burst_out = zeroExtend(burst_bytes);
//		//				Bit#(bytes_min) lv_zeroes= 0;
//		//				in_burst[(i+1)*v_bytes_min-1:i*v_bytes_min] = lv_zeroes;
//		//				lv_data_out = data[(i+1)*v_data_out-1:i*v_bytes_out];	
//		//			end
//		//		end
//		//	end
//		//	return tuple3(truncate(burst_out), in_burst, truncate(lv_data_out));
//		//endfunction
//			
//			
//		//function Bit#(8) awlen_out(Bit#(8) awlen_in, Bit#(3) awsize);
//		//	Bit#(8) awlen_out;
//		//	Bit#(3) awsize_out = zeroExtend(fromInteger(data_width_out));
//		//	if(awsize <= awsize_out)
//		//		awlen_out = awlen_in;
//		//	else begin
//		//		awlen_out = zeroExtend(awsize/awsize_out);
//		//	end
//		//endfunction
//
//		mkConnection(bus1, s_xactor.axi_side);
//		mkConnection(m_xactor.axi_side, bus2);
//
//		// These rule will receive the read request from the AXI4 fabric and pass it on to the AXI4Lite fabric.
//		// If the request is a burst then they are broken down to individual axi4lite read requests. These
//		// are carried out in the next rule. 
//		rule capture_read_requests_from_Axi4(rd_state==RegularReq);
//			let request<-pop_o(s_xactor.o_rd_addr);
//			Bit#(3) child_bus_arsize = request.arsize;
//			rg_parent_burst_awlen <= request.arlen;
//			if(valueOf(bytes_in) > valueOf(bytes_out)) begin 
//				if(request.arsize>fromInteger(v_size_out)) begin
//					Bit#(8) lv_awlen = 1;
//					lv_awlen = lv_awlen << request.arsize;
//					lv_awlen = lv_awlen >> fromInteger(v_size_out);
//					lv_awlen = lv_awlen-1;
//					rd_state <= BurstReq;
//					request.arlen=lv_awlen;	
//					request.arsize=fromInteger(v_size_out);
//					rg_rd_burst_metadata <= tuple2(lv_awlen, fromInteger(v_size_out));
//				end
//			end
//			ff_rd_addr.enq(request);
//			rg_read_packet<=request;
//			sync_rdburst_value<=request.arlen;
//		endrule
//		// In case a read-burst request is received on the fast bus, then the bursts have to broken down into
//		// individual slow-bus read requests. 
//		// This is rule is fired after the first read-burst request is sent to the slow_bus. This rule will continue to 
//		// fire as long as the slow bus has capacity to receive a new request and the burst is not complete.
//		// the difference between the each individual requests on the slow bus is only the address. All other 
//		// parameters remain the same.
//		rule generate_bust_read_requests(rd_state==BurstReq);
//			let request=rg_read_packet;
//			request.araddr=burst_address_generator(request.arlen, request.arsize, request.arburst,request.araddr);
//			request.arlen = tpl_1(rg_rd_burst_metadata);
//			request.arsize = tpl_2(rg_rd_burst_metadata);
//			rg_read_packet<=request;
//			ff_rd_addr.enq(request);
//			if(request.arlen==request_counter)begin
//				request_counter<=0;
//				if(rg_parent_burst_awlen==0)
//					rd_state <= RegularReq;
//				else
//					rg_parent_burst_awlen <= rg_parent_burst_awlen -1;
//			end
//			else
//				request_counter<=request_counter+1;
//		endrule
//
//		rule send_read_request_on_slow_bus;
//			let request=ff_rd_addr.first;
//			ff_rd_addr.deq;
//		 	request = AXI4_Rd_Addr {araddr: request.araddr, 
//																	aruser: request.aruser,
//																	arsize: request.arsize,
//																	arlen : request.arlen,
//																	arburst : request.arburst,
//																	arid : request.arid }; 
//																	// arburst: 00-FIXED 01-INCR 10-WRAP
//   	  m_xactor.i_rd_addr.enq(request);	
//			rd_id<=request.arid;
//		endrule
//		// This rule will capture the write request from the AXI4 fabric and pass it on to the AXI4Lite fabric.
//		// In case of burst requests, they are broken down to individual requests of axi4lite writes. Care
//		// needs to be taken when writes are of different sizes in settin the write-strobe correctly.
//		rule capture_write_requests_from_Axi4(wr_state==RegularReq);
//			let wr_addr_req  <- pop_o (s_xactor.o_wr_addr);
//	      let wr_data_req  <- pop_o (s_xactor.o_wr_data);
//			ff_wr_addr.enq(wr_addr_req);
//			ff_wr_data.enq(wr_data_req);
//			//rg_write_packet<=wr_addr_req;
//      //sync_wrburst_value <= wr_addr_req.awlen;
//			if(wr_addr_req.awlen!=0) begin
//				wr_state<=BurstReq;
//			end
//			`ifdef verbose $display($time,"\tAXIBRIDGE: Write Request"); `endif
//			`ifdef verbose $display($time,"\tAddress Channel :",fshow(wr_addr_req)); `endif
//			`ifdef verbose $display($time,"\tData Channel :",fshow(wr_data_req)); `endif
//		endrule
//		// In case a write-burst request is received on the fast bus, then the bursts have to broken down into
//		// individual slow-bus write requests. 
//		// This is rule is fired after the first write-burst request is sent to the slow_bus. This rule will continue to 
//		// fire as long as the slow bus has capacity to receive a new request and the burst is not complete i.e.
//		// fast bust xactor does not send wlast asserted.
//		// The difference between the each individual requests on the slow bus is only the address. All other 
//		// parameters remain the same.
//		rule generate_bust_write_requests(wr_state==BurstReq);
//			//let request=rg_write_packet;
//			//request.awaddr=burst_address_generator(request.awlen, request.awsize, request.awburst,request.awaddr);
//	      let wr_data_req  <- pop_o (s_xactor.o_wr_data);
//			//ff_wr_addr.enq(request);
//			ff_wr_data.enq(wr_data_req);
//			//rg_write_packet<=request;
//			if(wr_data_req.wlast)begin
//				wr_state<=RegularReq;
//			end
//			`ifdef verbose $display($time,"\tAXIBRIDGE: Burst Write Request"); `endif
//			`ifdef verbose $display($time,"\tData Channel1 :",fshow(wr_data_req)); `endif
//		endrule
//		
//		(*mutually_exclusive="send_write_request_on_child_bus, rl_burst_linear"*)
//		rule send_write_request_on_child_bus(!rg_wr_data_burst);
//			let wr_addr_req  = ff_wr_addr.first;
//	    let wr_data_req  = ff_wr_data.first;
//			Bit#(8) lv_awlen= wr_addr_req.awlen;
//			let lv_awsize = wr_addr_req.awsize;
//			if(valueOf(bytes_in) > valueOf(bytes_out) && wr_addr_req.awsize>fromInteger(v_size_out)) begin
//					lv_awlen = 1;
//					lv_awlen = lv_awlen << wr_addr_req.awsize;
//					lv_awlen = lv_awlen >> fromInteger(v_size_out);
//					lv_awlen = lv_awlen-1;
//					rg_tier2_burst <= lv_awlen; //TODO this depends on the number of requests in flight
//					rg_wr_child_burst <= True;
//					rg_wr_data_burst <= True;	
//					rg_parent_awlen <= wr_addr_req.awlen;
//			end
//			else if(lv_awlen!=0) begin
//				rg_linear_burst <= True;
//				ff_wr_addr.deq;
//			end
//			else begin
//				ff_wr_addr.deq;
//			end
//			ff_wr_data.deq;
//			rg_child_awlen <= lv_awlen;
//			
//			if(wr_addr_req.awsize > fromInteger(v_size_out))
//				lv_awsize = fromInteger(v_size_out);
//			Tuple3#(Bit#(bytes_out), Bit#(bytes_in), Bit#(data_width_out)) strb_mod;
//			strb_mod = strb_out(wr_data_req.wstrb, wr_data_req.wdata);
//			Bit#(bytes_out) lv_strb_out = tpl_1(strb_mod);
//			Bit#(bytes_in) lv_strb_in = tpl_2(strb_mod);
//			Bit#(data_width_out) lv_data = tpl_3(strb_mod);
//			
//			//let lv_awlen_out = awlen_out(wr_addr_req.awlen, wr_addr_req.awsize);
//			let aw = AXI4_Wr_Addr {awaddr: wr_addr_req.awaddr, 
//														 awuser:0, 
//														 awlen : lv_awlen,
//														 awsize: lv_awsize,
//														 awburst : wr_addr_req.awburst,
//														 awid : wr_addr_req.awid}; // arburst: 00-FIXED 01-INCR 10-WRAP
//
//			let w  = AXI4_Wr_Data {wdata:  lv_data, 
//														 wstrb: lv_strb_out,
//														 wid : wr_data_req.wid,
//														 wlast : lv_awlen==0};
//
//			wr_addr_req.awaddr=burst_address_generator(wr_addr_req.awlen, wr_addr_req.awsize, 
//																													wr_addr_req.awburst,wr_addr_req.awaddr);
//			wr_addr_req.awlen = lv_awlen;
//			wr_addr_req.awsize = lv_awsize;
//			rg_write_packet<=wr_addr_req;
//
//			wr_data_req.wstrb = lv_strb_in;
//			rg_data_packet <= wr_data_req;
//			m_xactor.i_wr_addr.enq(aw);
//			m_xactor.i_wr_data.enq(w);
//			wr_id <= wr_addr_req.awid;
//		endrule
//
//		rule rl_initiate_burst_child(!rg_wr_child_burst && rg_wr_data_burst);
//			let wr_addr_req = ff_wr_addr.first;
//			let wr_data_req = ff_wr_data.first;
//			let request=rg_write_packet;
//			
//			Tuple3#(Bit#(bytes_out), Bit#(bytes_in), Bit#(data_width_out)) strb_mod;
//			strb_mod = strb_out(wr_data_req.wstrb, wr_data_req.wdata);
//			Bit#(bytes_out) lv_strb_out = tpl_1(strb_mod);
//			Bit#(bytes_in) lv_strb_in = tpl_2(strb_mod);
//			Bit#(data_width_out) lv_data = tpl_3(strb_mod);
//			//let lv_awlen_out = awlen_out(wr_addr_req.awlen, wr_addr_req.awsize);
//			let aw = AXI4_Wr_Addr {awaddr: request.awaddr, 
//														 awuser:0, 
//														 awlen : request.awlen,
//														 awsize: request.awsize,
//														 awburst : wr_addr_req.awburst,
//														 awid : wr_addr_req.awid}; // arburst: 00-FIXED 01-INCR 10-WRAP
//			let w  = AXI4_Wr_Data {wdata:  lv_data, 
//														 wstrb: lv_strb_out,
//														 wid : wr_data_req.wid,
//														 wlast : False};
//			request.awaddr=burst_address_generator(request.awlen, request.awsize, 
//																														request.awburst,request.awaddr);
//			rg_write_packet<=request;
//			wr_data_req.wstrb = lv_strb_in;
//			rg_data_packet <= wr_data_req;
//			rg_child_awlen <= request.awlen;
//			if(rg_parent_awlen!=0)
//				rg_parent_awlen <= rg_parent_awlen - 1;
//			else
//				ff_wr_addr.deq;
//			ff_wr_data.deq;
//			m_xactor.i_wr_addr.enq(aw);
//			m_xactor.i_wr_data.enq(w);
//		endrule
//
//		rule rl_burst_child(rg_wr_child_burst && rg_wr_data_burst);
//			let wr_data_req = ff_wr_data.first;
//			let data_request = rg_data_packet;
//			Tuple3#(Bit#(bytes_out), Bit#(bytes_in), Bit#(data_width_out)) strb_mod;
//			strb_mod = strb_out(rg_data_packet.wstrb, rg_data_packet.wdata);
//			Bit#(bytes_out) lv_strb_out = tpl_1(strb_mod);
//			Bit#(bytes_in) lv_strb_in = tpl_2(strb_mod);
//			Bit#(data_width_out) lv_data = tpl_3(strb_mod);
//			
//			data_request.wstrb = lv_strb_in;
//			rg_data_packet <= data_request;
//			
//			Bool lv_wlast = False; 
//			if(rg_child_awlen!=0)
//				rg_child_awlen <= rg_child_awlen - 1;
//			else begin
//				ff_wr_data.deq;
//				rg_wr_child_burst <= False;
//				if(rg_parent_awlen==0)
//					rg_wr_data_burst <= False;
//			end
//
//			let w  = AXI4_Wr_Data {wdata:  lv_data, 
//														 wstrb: lv_strb_out,
//														 wid : wr_data_req.wid,
//														 wlast : lv_wlast};
//			//wr_data_req.astrb = lv_strb_in;
//			//rg_data_request <= wr_data_req;
//			m_xactor.i_wr_data.enq(w);
//		endrule
//
//		rule rl_burst_linear(rg_linear_burst);
//			let wr_data_req = ff_wr_data.first;
//			//let {lv_strb_out, lv_strb_in, lv_data} = strb_out(rg_data_packet.wstrb,
//			//																										 rg_data_packet.wdata);	
//			//data_request.wstrb = lv_strb_in;
//			//rg_data_packet <= data_request;
//			Bit#(TAdd#(data_width_out, data_width_in)) lv_data = zeroExtend(wr_data_req.wdata);	
//			Bit#(TAdd#(TDiv#(data_width_out,8), TDiv#(data_width_in,8))) 
//																													lv_strb = zeroExtend(wr_data_req.wstrb);
//			if(rg_child_awlen!=0)
//				rg_child_awlen <= rg_child_awlen - 1;
//			else begin
//				ff_wr_data.deq;
//				rg_linear_burst <= False;
//			end
//
//			let w  = AXI4_Wr_Data {wdata:  truncate(lv_data), 
//														 wstrb: truncate(lv_strb),
//														 wid : wr_data_req.wid,
//														 wlast : wr_data_req.wlast};
//			//wr_data_req.astrb = lv_strb_in;
//			//rg_data_request <= wr_data_req;
//			m_xactor.i_wr_data.enq(w);
//		endrule
//
//		// This rule forwards the read response from the AXI4 to the AXI4 fabric.
//		rule capture_read_responses;
//			let response <- pop_o (m_xactor.o_rd_data);
//			//AXI4_Resp rresp= case(response.rresp)
//			//	AXI4_LITE_OKAY  : AXI4_OKAY;
//			//	AXI4_LITE_EXOKAY: AXI4_EXOKAY;
//			//	AXI4_LITE_SLVERR: AXI4_SLVERR;
//			//	AXI4_LITE_DECERR: AXI4_DECERR;
//			//	default: AXI4_SLVERR; endcase;
//			Bit#(TAdd#(data_width_out, data_width_in)) temp_resp = zeroExtend(response.rdata);//TODO need to dulicate this stuff
//			if(rg_sync_rd_child_burst) begin
//				Bit#(data_width_in) resp_collect = read_data_resp(response.rdata, rg_data_resp_collect);  
//				//Bit#(TLog#(data_width_in)) data_in_bytes = (1<<arsize_req)*8;
//				//response.rdata = resp_collect[data_in_bytes-1:0];
//				rg_data_resp_collect <= resp_collect;
//				AXI4_Rd_Data#(data_width_in,userspace) r = AXI4_Rd_Data {rresp: response.rresp, 
//																																	rdata: resp_collect,
//																																	rlast:rd_response_counter==sync_rdburst_value, 
//																																	ruser: 0, rid:rd_id};
//				if(response.rlast) begin
//					ff_rd_resp.enq(r);
//				end
//			end
//			else begin
//				AXI4_Rd_Data#(data_width_in,userspace) r = AXI4_Rd_Data {rresp: response.rresp, 
//																																	rdata: truncate(temp_resp),
//																																	rlast:rd_response_counter==sync_rdburst_value, 
//																																	ruser: 0, rid:rd_id};
//				if(rd_response_counter==sync_rdburst_value)
//					rd_response_counter<=0;
//				else begin
//					rd_response_counter<=rd_response_counter+1;
//				end
//				ff_rd_resp.enq(r);
//			end
//		endrule
//		rule send_read_response_on_fast_bus;
//			ff_rd_resp.deq;
//			s_xactor.i_rd_data.enq(ff_rd_resp.first);
//		endrule
//		rule capture_write_responses;
//			let response<-pop_o(m_xactor.o_wr_resp);
//			//AXI4_Resp bresp= case(response.bresp)
//			//	AXI4_LITE_OKAY  : AXI4_OKAY;
//			//	AXI4_LITE_EXOKAY: AXI4_EXOKAY;
//			//	AXI4_LITE_SLVERR: AXI4_SLVERR;
//			//	AXI4_LITE_DECERR: AXI4_DECERR;
//			//	default: AXI4_SLVERR; endcase;
//			let b = AXI4_Wr_Resp {bresp: response.bresp, buser:0, bid:wr_id};
//      if(wr_response_counter == rg_tier2_burst) begin
//			  ff_wr_resp.enq(b);
//        wr_response_counter <= 0;
//      end
//      else
//        wr_response_counter <= wr_response_counter + 1;
//		endrule
//		rule send_write_response_on_fast_bus;
//			ff_wr_resp.deq;
//			s_xactor.i_wr_resp.enq(ff_wr_resp.first);
//		endrule
//		//interface axi_slave=s_xactor.axi_side;
//		//interface axi4_lite_master=m_xactor.axi_side;
//	endmodule	
//endinstance
endpackage

