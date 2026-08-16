/*
	Threads based on ISO metric threads: https://en.wikipedia.org/wiki/ISO_metric_screw_thread
	Default parameter values will result in ISO threads
*/
use <../scad-turtlepath/TurtlePath3d.scad>;
use <../scad-turtlepath/TurtlePath3d_Sweep.scad>;

for(k=["ext","int"])
Threads(
	kind=k,
	thread_d=20,
	pitch=3,
	starts=3,
	thread_len=5,
	iclr=0.5,
	eclr=0.1,
	skew_w=+0.8,
	//fillet_in=false,
	//fillet_out=false,
	//inner_wall_d=12,
	//outer_wall_d=25,
);

module Threads(
	// "ext" for external threads, "int" for internal threads
	kind="ext", // int|ext
	// nominal outside external thread diameter
	thread_d=20,
	// thread pitch in mm
	pitch=2,
	// how many starts to use
	starts=1,
	// total thread length from peak to peak
	thread_len=20,
	// external thread clearance (reduce outer diameter)
	eclr=0.1,
	// internal thread clearance (increase inner diameter)
	iclr=0.1,
	// extertal thread tip flat width in percent of the pitch
	eflat_w_pct=1/8,
	// internal thread tip flat width in percent of the pitch
	iflat_w_pct=1/4,
	// thread height between flats in percent of the pitch
	thread_h_pct=5/8,
	// skew weight to split the remainder of the pitch between the leading and trailing slopes
	// +1: for external threads: all of it on leading slope, trailing slope horizontal; internal threads: opposite
	// -1: for external threads: all of it on trailing slope, leading slope horizontal; internal threads: opposite
	skew_w=0,
	// fillet in the start of the thread
	fillet_in=true,
	// fillet out the end of the thread
	fillet_out=true,
	// for external threads, include inner pipe wall with given inner diameter and length == thread_len+2*pitch
	inner_wall_d=0,
	// for internal threads, include outer pipe wall with given outer diameter and length == thread_len+2*pitch
	outer_wall_d=0,
	// prefix for assertions
	debug_prefix="VThreads")
assert(is_string(kind) && (kind=="ext" || kind=="int"),debug_prefix)
assert(is_num(thread_d) && thread_d>0,debug_prefix)
assert(is_num(pitch) && pitch>0,debug_prefix)
assert(is_num(starts) && starts>0 && starts==round(starts),debug_prefix)
assert(is_num(thread_len) && thread_len>0,debug_prefix)
assert(is_num(eclr) && eclr>=0,debug_prefix)
assert(is_num(iclr) && iclr>=0,debug_prefix)
assert(is_num(eflat_w_pct) && eflat_w_pct>0,debug_prefix)
assert(is_num(iflat_w_pct) && iflat_w_pct>0,debug_prefix)
assert(eflat_w_pct+iflat_w_pct<1,debug_prefix)
assert(is_num(thread_h_pct) && thread_h_pct>0 && thread_h_pct<1,debug_prefix)
assert(is_num(skew_w) && abs(skew_w)<=1,debug_prefix)
assert(is_bool(fillet_in),debug_prefix)
assert(is_bool(fillet_out),debug_prefix)
assert(is_num(inner_wall_d) && inner_wall_d>=0,debug_prefix)
assert(is_num(outer_wall_d) && outer_wall_d>=0,debug_prefix)
let(isext=kind=="ext",
	pt_step_a=5,

	pt_st=[thread_d/2,0,0],
	pt_a=360*thread_len/(pitch*starts),
	pt_n=ceil((360/pt_step_a)*thread_len/(pitch*starts)),
	thr_pt=TurtlePath3d(debug_prefix=str(debug_prefix," path"),start_pt=pt_st,steps=[
		function(acc,dp) tp3d_Turn(acc,dp,a=pt_a,r=thread_d/2,n=pt_n,dn=thread_len),
	]),

	thr_iflat_w=pitch*iflat_w_pct,
	thr_eflat_w=pitch*eflat_w_pct,
	thr_h=cos(30)*pitch,
	thr_hsz=thr_h*thread_h_pct,
	thr_id=(isext?-eclr:+iclr)+thread_d-thr_hsz*2,
	thr_ir=thr_id/2,
	thr_od=(isext?-eclr:+iclr)+thread_d,
	thr_or=thr_od/2,
	thr_slope_total_w=pitch*(1-iflat_w_pct-eflat_w_pct),
	thr_slope_lead_w=thr_slope_total_w*(skew_w+1)/2,
	thr_slope_trail_w=thr_slope_total_w*(1-skew_w)/2,
	thr_slope_lead_a=atan2(thr_slope_lead_w,thr_hsz),
	thr_slope_trail_a=atan2(thr_slope_trail_w,thr_hsz),
	threadform_prf=function(fillet_pct) // fillet_pct<1 ? fillet : normal
		assert(fillet_pct>=0 && fillet_pct<=1,debug_prefix)
		let(dp=str(debug_prefix," k=",kind," profile fillet_pct=",fillet_pct),
			pct=fillet_pct*0.99+0.01,
			hsz=pct*thr_hsz,
			id=(isext ? thr_id : thr_od-hsz*2),
			ir=id/2,
			od=(isext ? thr_id+hsz*2 : thr_od),
			or=od/2,
			pr=isext
			? TurtlePath3d(debug_prefix=dp,
				start_pt=[(id-thread_d)/2-thr_iflat_w/3,thr_eflat_w/2+pct*thr_slope_trail_w+thr_iflat_w/3,0],
				start_hv=[cos(-45),sin(-45),0],
				steps=[
					function(acc,dp) tp3d_Straight(acc,dp,d=(thr_iflat_w/3)/cos(45)),
					function(acc,dp) tp3d_Pivot(acc,dp,a_horz=+45-thr_slope_trail_a),
					function(acc,dp) tp3d_Straight(acc,dp,d=hsz/cos(thr_slope_trail_a)),
					function(acc,dp) tp3d_Pivot(acc,dp,a_horz=thr_slope_trail_a-90),
					function(acc,dp) tp3d_Straight(acc,dp,d=thr_eflat_w),
					function(acc,dp) tp3d_Pivot(acc,dp,a_horz=-90+thr_slope_lead_a),
					function(acc,dp) tp3d_Straight(acc,dp,d=hsz/cos(thr_slope_lead_a)),
					function(acc,dp) tp3d_Pivot(acc,dp,a_horz=-thr_slope_lead_a+45),
					function(acc,dp) tp3d_Straight(acc,dp,d=(thr_iflat_w/3)/cos(45)),
				])
			: TurtlePath3d(debug_prefix=dp,
				start_pt=[(od-thread_d)/2+thr_eflat_w/3,-pct*thr_slope_total_w/2-thr_iflat_w/2-thr_eflat_w/3,0],
				start_hv=[cos(135),sin(135),0],
				steps=[
					function(acc,dp) tp3d_Straight(acc,dp,d=(thr_eflat_w/3)/cos(45)),
					function(acc,dp) tp3d_Pivot(acc,dp,a_horz=45-thr_slope_trail_a),
					function(acc,dp) tp3d_Straight(acc,dp,d=hsz/cos(thr_slope_trail_a)),
					function(acc,dp) tp3d_Pivot(acc,dp,a_horz=thr_slope_trail_a-90),
					function(acc,dp) tp3d_Straight(acc,dp,d=thr_iflat_w),
					function(acc,dp) tp3d_Pivot(acc,dp,a_horz=-90+thr_slope_lead_a),
					function(acc,dp) tp3d_Straight(acc,dp,d=hsz/cos(thr_slope_lead_a)),
					function(acc,dp) tp3d_Pivot(acc,dp,a_horz=-thr_slope_lead_a+45),
					function(acc,dp) tp3d_Straight(acc,dp,d=(thr_eflat_w/3)/cos(45)),
				]))
		tp3d_GetPathXY(pr),
	main_pr=threadform_prf(1),
	thr_prf=function(i)
		let(pct = fillet_in && i<=5 ? i/5
				: fillet_out && (pt_n-i)<=5 ? (pt_n-i)/5
				: 1)
		pct==1 ? main_pr : threadform_prf(sin(90*pct)),

	pipe_enabled=isext
		? inner_wall_d>0 && inner_wall_d<thr_id-2*thr_iflat_w/3
		: outer_wall_d>thr_od+2*thr_eflat_w/3,
	pipe_pr= !pipe_enabled ? [] :
		let(ix= isext ? inner_wall_d/2-thread_d/2 : thr_od/2-thread_d/2+thr_eflat_w/6,
			ox= isext ? thr_id/2-thread_d/2-thr_iflat_w/6 : outer_wall_d/2-thread_d/2,
			ty=+pitch+thread_len,
			by=-pitch)
		[
			[ix,ty],
			[ox,ty],
			[ox,by],
			[ix,by],
		],
	pipe_pt= !pipe_enabled ? []
		: TurtlePath3d(debug_prefix=str(debug_prefix," pipe wall path"),drop_last=true,start_pt=pt_st,steps=[
			function(acc,dp) tp3d_Turn(acc,dp,a=360,r=pt_st.x,n=ceil(360/pt_step_a)),
		]),
	)
{
	rotate([0,0,isext?0:180/starts])
	for(s=[1:starts])
	rotate([0,0,(s-1)*360/starts])
	TurtlePath3d_Sweep(debug_prefix=debug_prefix,profile=thr_prf,path=thr_pt);

	if(pipe_enabled)
	TurtlePath3d_Sweep(debug_prefix=str(debug_prefix," pipe wall"),profile=pipe_pr,path=pipe_pt,closure=["torus"]);
}
