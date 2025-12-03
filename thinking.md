Report structure + process log:

In general: 
We start out by writing a verifier Python script that includes the following math:
[explain math from python script]

Since we are running this math multiple times to fine-tune our gains, it is best to spend the time to write code and automate the process of recalculating.

for x controller:

we start with finding out that to roughly meet risetime we need a gain of 0.065 
0.065 is not effective [SHOW MY MATH REASONING], takes longer than sim to get to command value and has ss err
we increase it 1.0, and it works but it still violates rise and settling time [works but too slow]
--> we need to make the proprtional controller much larger for a quicker response

We increase Kp to 2 --> We sligthly exceed minimum rise time. So we find that around 1.5 perfectly hits our rise time. Now we aren't settling fast enough though, so the system needs to be more damped.

damping ratio increases w damping coeff; derivate control can help us dampen the system (rotate locus branches to the left). Refer to ziegler nichols and choose Kd ~= 10% of Kp => 1.5 * 0.1 = 0.15. So we can have initial values of 1.5 Kp and .15 Kd for Kx (still 0 Ki)

Our rise time is ever so slightly high at about 1.29s, so we increase Kp more. We leave Kd as-is first (Kd will decrease our rise time).
Pushing Kp up to 1.75, we get a perfect rise time! But we have made our settling time ever so slightly high, so we choose to increase Kd to the new benchmark of 10% of 1.75 => .175.

I am at a good spot with a working controller, and I realize that the actual fine-tuning and iteration process (after using controls theory to get to a close range within the answer) may be best solved with a tool from numerical methods.

For Y-controller: 

I start with deriving the equations from the block diagram. I end up with a second order system in the denominator (characteristic eqn), which I can use to find initial values for Ky and Kphi. I introduce a midpoint value of tr = 3.25 (in the middle of the envelope) to get Wn, and introduce the overshoot equation, and an overshoot constraint of 10% (also from project reqs) to get a value for the damping ratio.

Once I have these values, I have two unkowns and two equations (Kphi and Ky). I set Kphi = Kphi (solved for Kphi in both eqns), and solve for Ky, getting a value of -0.00003006. I solve for Kphi and get a value 0 -0.99

After running the program with these, I see that my y-locn is chaning too slowly, because of my extremely small order of magnitude. I go up to -0.0001 and test with satisfactory results; now however, I have overshoot on the y-direction, which may be tuned out in Kphi. I reduce Kphi by order of 0.05 from ~-1.0 to .995, .98, .985, .980... .980 works slightly slow, so I go to a middle point of .9825. It works great!

I am slightly over some benchmark values, and I resort to the previously mentioned numerical methods tool to fine-tune all of my controller values (for y, phi and x)


Fine-tuning with optimization: new section

I resort to a tool I learned in numerical methods; optimization! I use a Claude CLI agent to discuss different optimization options, and although I tried multivariable gradient descent, claude suggests [simpler random serach in optimize_gains.py]. I go ahead with the suggestion and build the program. It works [explain it in basic ish terms, with the math].

Using the program [run the program again!] I fine-tune the gains and end up wth perfect benchmarks!

I write a small, simple conclusion

I am done!

left to do:

--> upload ALL the stuff to a github repo in case prof wants to look at it
--> Make a simple .tex report (claude can help me format later on)
--> include figures from MATLAB output (put them in images folder in root)

nice!