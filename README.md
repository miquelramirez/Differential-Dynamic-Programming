# Differential-Dynamic-Programming

This repo includes starter code and a brief overview on differential dynamic programming (DDP) for operating your own autonomous helicopter! Well, at least kickstarting an implementation of this paper, [An Application of Reinforcement Learning to Aerobatic Helicopter Flight](http://heli.stanford.edu/papers/nips06-aerobatichelicopter.pdf).

* [Getting Started](https://github.com/SioKCronin/Differential-Dynamic-Programming/blob/master/Getting%20Started.ipynb)
* [DDP](https://github.com/SioKCronin/Differential-Dynamic-Programming/tree/master/DDP)

## Paper Highlights

* The protocol follows three stages: 1) gather human pilot data, 2) find a controller in simulation based on the model developed from the training data, 2) test on helicopter (iterate as necessary)
* Four maneuvers were tested: Flip, Roll, Tail-in Funnel, Nose-in Funnel
* Flips and rolls presented unique noise challenges, so in their training they maneuvered full sweep arcs that approached flipping but didn't go all the way to inversion. Q and R cost matrices were coded by hand to create a controller that would flip indefinitely in their simulator.

## Resources

* Reinforcement Learning (Sutton & Barto) - nice introduction to dynamic programming
* [Differential Dynamic Programming for Optimal Estimation](https://www.cc.gatech.edu/~dellaert/pubs/Kobilarov15icra.pdf) - helpful article
* Steve Brunton's awesome [Control Bootcamp](https://www.youtube.com/channel/UCm5mt-A4w61lknZ9lCsZtBw)
* Brian Douglas' [playlist](https://www.youtube.com/watch?v=oBc_BHxw78s&list=PLUMWjy5jgHK1NC52DXXrriwihVrYZKqjk)
* Emma Brunskill's [talks](https://www.youtube.com/watch?v=fIKkhoI1kF4&list=PLAsrlO2SCuzBVqN6V1CQSL4VdaGv7LawW) at Simons Institute
* [Deep RL Bootcamp](https://sites.google.com/view/deep-rl-bootcamp/lectures)
