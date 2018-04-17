# Paper implementation

# Conroller design

'''
1. Noise free testing of open-loop data with DDP (feedforward controller from classic control)
2. DDP for flight controller, with inputs as deviations from the nominal 
open-loop (penalizes only unplanned changes in the contorl inputs)
'''

# Inverse RL algorithm (to learn from human flight)

'''
"Iteratively provide us with reward weights that result in policies that bring us closer to the expert"
But hand chose weights as well, by increasing/decreasing the weights for those features
that stood out as mostly different from the expert
'''


