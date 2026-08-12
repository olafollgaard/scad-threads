/*
	60 degrees V threads
*/
use <../scad-turtlepath/TurtlePath3d.scad>;
use <../scad-turtlepath/TurtlePath3d_Sweep.scad>;

//translate([-10,0,0])
for(g=["m","f"])
VThreads(
	thread_g=g,
	thread_d=20,
	wall_ith=0.1,
	pitch=2,
	trunc=0.2,
	starts=2,
	length=2,
	clr=0.1,
	//fillet_in=false,
	//fillet_out=false,
	//male_id=15,
	//female_od=21.5,
);

module VThreads(
	// "m" for male threads, "f" for female threads
	thread_g="m", // m|f
	// thread diameter - measured as the outside diameter of the male thread, disregarding truncation and clearance
	thread_d=20,
	// thread 'wall' intersection thickness
	wall_ith=0.1,
	// thread pitch in mm
	pitch=2,
	// truncation of the thread peaks
	trunc=0.2,
	// how many starts to use
	starts=1,
	// total length from peak to peak
	length=20,
	// clearance
	clr=0.1,
	// fillet in the start of the thread
	fillet_in=true,
	// fillet out the end of the thread
	fillet_out=true,
	// for male threads, include pipe wall with given inner diameter
	male_id=0,
	// for female threads, include pipe wall with given outer diameter
	female_od=0,
	// prefix for assertions
	debug_prefix="VThreads")
assert(is_string(thread_g) && (thread_g=="m" || thread_g=="f"),debug_prefix)
assert(is_num(wall_ith) && wall_ith>0,debug_prefix)
assert(is_num(pitch) && pitch>0,debug_prefix)
assert(is_num(trunc) && trunc>0,debug_prefix)
assert(is_num(starts) && starts>0 && starts==round(starts),debug_prefix)
assert(is_num(length) && length>0,debug_prefix)
assert(is_num(clr) && clr>=0,debug_prefix)
assert(is_num(male_id) && male_id>=0,debug_prefix)
assert(is_num(female_od) && female_od>=0,debug_prefix)
let(male=thread_g=="m",
	step_a=5,

	pt_st=[thread_d/2,0,0],
	pt_a=360*length/(pitch*starts),
	pt_n=ceil((360/step_a)*length/(pitch*starts)),
	thr_pt=TurtlePath3d(debug_prefix=str(debug_prefix," path"),start_pt=pt_st,steps=[
		function(acc,dp) tp3d_Turn(acc,dp,a=pt_a,r=thread_d/2,n=pt_n,dn=length),
	]),

	threadform_prf=function(sz)
		let(dp=str(debug_prefix," g=",thread_g," profile sz=",sz),
			pr=male
			? TurtlePath3d(debug_prefix=dp,
				start_pt=[-pitch*cos(30)-wall_ith,sz/2-clr/20,0],start_hv=[1,0,0],
				steps=[
					function(acc,dp) tp3d_AddLabel(acc,dp,"st"),
					function(acc,dp) tp3d_Straight(acc,dp,d= wall_ith + cos(30)*clr/10 - clr/2 ),
					function(acc,dp) tp3d_Pivot(acc,dp,a_horz=-30),
					function(acc,dp) tp3d_Straight(acc,dp,d= sz-trunc-clr/10 ),
					function(acc,dp) tp3d_Mirror(acc,dp,stop="st",mpt=[0,0,0],mnv=[0,1,0]),
				])
			: TurtlePath3d(debug_prefix=dp,
				start_pt=[trunc*cos(30)+clr/2-sz*cos(30),trunc/2,0],start_hv=[cos(30),sin(30),0],
				steps=[
					function(acc,dp) tp3d_AddLabel(acc,dp,"st"),
					function(acc,dp) tp3d_Straight(acc,dp,d= sz-trunc-clr/10 ),
					function(acc,dp) tp3d_Pivot(acc,dp,a_horz=-30),
					function(acc,dp) tp3d_Straight(acc,dp,d= cos(30)*clr/10+wall_ith-clr/2 ),
					function(acc,dp) tp3d_Mirror(acc,dp,stop="st",mpt=[0,0,0],mnv=[0,1,0]),
				]))
		tp3d_GetPathXY(pr),
	main_pr=threadform_prf(pitch),
	thr_prf=function(i)
		let(pct = fillet_in && i<=5 ? i/5
				: fillet_out && (pt_n-i)<=5 ? (pt_n-i)/5
				: 1,
			min_sz=trunc+1.1*clr/10)
		pct==1 ? main_pr : threadform_prf(min_sz+(pitch-min_sz)*sin(90*pct)),

	pipe_enabled=male
		? male_id>0 && male_id<thread_d-2*(pitch*cos(30)+wall_ith)
		: female_od>thread_d+2*wall_ith,
	pipe_pr= !pipe_enabled ? [] :
		let(ix= male ? male_id/2-thread_d/2 : clr/2-trunc*cos(30),
			ox= male ? -(pitch-trunc)*cos(30)-clr/2 : female_od/2-thread_d/2)
		[
			[ix,length+pitch/2],
			[ox,length+pitch/2],
			[ox,-pitch/2],
			[ix,-pitch/2],
		],
	pipe_pt= !pipe_enabled ? []
		: TurtlePath3d(debug_prefix=str(debug_prefix," pipe wall path"),drop_last=true,start_pt=pt_st,steps=[
			function(acc,dp) tp3d_Turn(acc,dp,a=360,r=pt_st.x,n=ceil(360/step_a)),
		]),
	)
{
	rotate([0,0,male?0:180/starts])
	for(s=[1:starts])
	rotate([0,0,(s-1)*360/starts])
	TurtlePath3d_Sweep(debug_prefix=debug_prefix,profile=thr_prf,path=thr_pt);

	if(pipe_enabled)
	TurtlePath3d_Sweep(debug_prefix=str(debug_prefix," pipe wall"),profile=pipe_pr,path=pipe_pt,closure=["torus"]);
}
