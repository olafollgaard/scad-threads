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
	fwall_th=0.2,
	pitch=2,
	trunc=0.2,
	starts=2,
	length=2,
	clr=0.1,
	//fillet_in=false,
	//fillet_out=false,
);

module VThreads(
	// "m" for male threads, "f" for female threads
	thread_g="m", // m|f
	// thread diameter - measured as the outside diameter of the male thread, disregarding truncation and clearance
	thread_d=20,
	// female thread outside 'wall' thickness
	fwall_th=0.2,
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
	// prefix for assertions
	debug_prefix="VThreads")
assert(is_string(thread_g) && (thread_g=="m" || thread_g=="f"),debug_prefix)
assert(is_num(pitch) && pitch>0,debug_prefix)
assert(is_num(trunc) && trunc>0,debug_prefix)
assert(is_num(starts) && starts>0 && starts==round(starts),debug_prefix)
assert(is_num(length) && length>0,debug_prefix)
assert(is_num(clr) && clr>=0,debug_prefix)
let(male=thread_g=="m",

	pt_st=[thread_d/2,0,0],
	pt_a=360*length/(pitch*starts),
	pt_n=ceil((360/5)*length/(pitch*starts)),
	pt=TurtlePath3d(debug_prefix=str(debug_prefix," path"),start_pt=pt_st,steps=[
		function(acc,dp) tp3d_Turn(acc,dp,a=pt_a,r=thread_d/2,n=pt_n,dn=length),
	]),

	threadform_prf=function(sz)
		let(dp=str(debug_prefix," ",thread_g," profile sz=",sz),
			pr=male
			? TurtlePath3d(debug_prefix=dp,
				start_pt=[-pitch,sz/2-trunc/2,0],
				steps=[
					function(acc,dp) tp3d_AddLabel(acc,dp,"st"),
					function(acc,dp) tp3d_Pivot(acc,dp,a_horz=-90),
					function(acc,dp) tp3d_Straight(acc,dp,d= trunc*cos(30) + (1-cos(30))*sz - clr/2 ),
					function(acc,dp) tp3d_Pivot(acc,dp,a_horz=-30),
					function(acc,dp) tp3d_Straight(acc,dp,d= sz-2*trunc ),
					function(acc,dp) tp3d_Mirror(acc,dp,stop="st",mpt=[0,0,0],mnv=[0,1,0]),
				])
			: TurtlePath3d(debug_prefix=dp,
				start_pt=[trunc*cos(30)+clr/2-sz*cos(30),trunc/2,0],
				steps=[
					function(acc,dp) tp3d_AddLabel(acc,dp,"st"),
					function(acc,dp) tp3d_Pivot(acc,dp,a_horz=-60),
					function(acc,dp) tp3d_Straight(acc,dp,d= sz-2*trunc ),
					function(acc,dp) tp3d_Pivot(acc,dp,a_horz=-30),
					function(acc,dp) tp3d_Straight(acc,dp,d= trunc*cos(30) + fwall_th - clr/2 ),
					function(acc,dp) tp3d_Mirror(acc,dp,stop="st",mpt=[0,0,0],mnv=[0,1,0]),
				]))
		tp3d_GetPathXY(pr),
	main_pr=threadform_prf(pitch),
	prf=function(i)
		let(pct = fillet_in && i<=5 ? i/5
				: fillet_out && (pt_n-i)<=5 ? (pt_n-i)/5
				: 1)
		pct==1 ? main_pr : threadform_prf(2.1*trunc+(pitch-2.1*trunc)*sin(90*pct)),
	)
{
	rotate([0,0,male?0:180/starts])
	for(s=[1:starts])
	rotate([0,0,(s-1)*360/starts])
	TurtlePath3d_Sweep(debug_prefix=debug_prefix,profile=prf,path=pt);
}
