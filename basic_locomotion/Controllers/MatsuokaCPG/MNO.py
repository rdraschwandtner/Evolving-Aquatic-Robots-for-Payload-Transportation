""" 
    Implements a Matsuoka Neural Oscillator.
"""

import random

class MNO(object):
    """ Matsuoka Neural Oscillator """

    def __init__(self,**kwargs):
    	""" Constructor for the class.

    	Arguments:
    		a   - strength of mutual inhibition
    		b   - degree of adaptation 
    		tau - inhibitory time constant
    		T   - time constant
    		dt  - time step size (integration resolution)

    	Attributes:
    		limit - numerical limit on the parameter a
    	"""
    	self.limit = 10.
    	self.c = 0.

        if(len(kwargs) > 1):
   			self.__set_params(**kwargs)
        else:
   			self.__rand_params(dt=kwargs['dt'])

    def __str__(self):
        """ To String method for the class. """
        return str(self.a)+","+str(self.b)+","+str(self.tau)+","+str(self.T)+"\n"

    def copy(self):
    	""" Copy constructor for the class """
    	return MNO(a=self.a,b=self.b,tau=self.tau,T=self.T,dt=self.dt)

    def __set_params(self,**kwargs):
    	""" Set the parameters from a neural oscillator to the provided values.

    	Note:
    		Should we perform parameter checking for provided values?
    	"""
    	self.a   = kwargs['a']
    	self.b   = kwargs['b']
    	self.tau = kwargs['tau']
    	self.T   = kwargs['T']
    	self.dt  = kwargs['dt']
    	self.__init_signals()

    def __rand_params(self,dt=1.):
    	""" Setup a neural oscillator with random parameter values.

    	Parameters should satisfy the equation:
    		1 + (tau/T) < a < (1 + b)

    	Returns:
    		tuple consisting of the a, b, tau, and T parameters
    	"""
    	self.dt  = dt

    	self.a   = random.random()*self.limit
    	self.b   = (self.a-1.)+(random.random()*(self.limit-self.a))
    	self.tau = random.random()*(self.a-1.)
    	self.T   = random.random()*(self.a-1.)

    	# Ensure that the parameters are the correct values.
    	self.__validate_params()
    	self.__init_signals()

    def __init_signals(self):
    	""" Initialize the signals inside the neural oscillator. 

    	Note: 
    		one of the signals must not be zero otherwise the system
    		will fail to oscillate.  From Matsuoka himself.
    	"""
    	self.y = [0.,0.]
    	self.x = [.1,0.]
    	self.v = [0.,0.]

    def step(self,c=0.):
    	""" Step the neural oscillator.

    	Arguments:
    		c - external input value, 0 means nothing incoming

    	Returns:
    		output of the CPG for that timestep 
    	"""
    	self.c = c
    	self.__integrate()
    	return self.get_output()

    def get_output(self):
    	""" Return the current output of the neural oscillator. """
    	return self.y[0] - self.y[1]

    def __integrate(self):
    	""" Integrate the state of the neural oscillator. """
    	a = self.evaluate(0.,MNODeriv())
    	b = self.evaluate(self.dt*0.5,a)
    	c = self.evaluate(self.dt*0.5,b)
    	d = self.evaluate(self.dt,c)

    	# neuron 1 signal derivatives
    	dx1 = 1./6. * (a.dx1 + 2.*(b.dx1 + c.dx1) + d.dx1)
    	dv1 = 1./6. * (a.dv1 + 2.*(b.dv1 + c.dv1) + d.dv1)

    	# neuron 2 signal derivatives
    	dx2 = 1./6. * (a.dx2 + 2.*(b.dx2 + c.dx2) + d.dx2)
    	dv2 = 1./6. * (a.dv2 + 2.*(b.dv2 + c.dv2) + d.dv2)

    	# update neuron 1 signals
    	self.x[0] = self.x[0] + (dx1 * self.dt)
    	self.v[0] = self.v[0] + (dv1 * self.dt)
    	self.y[0] = max(self.x[0], 0.)

    	# update neuron 2 signals
    	self.x[1] = self.x[1] + (dx2 * self.dt)
    	self.v[1] = self.v[1] + (dv2 * self.dt)
    	self.y[1] = max(self.x[1], 0.)

    def evaluate(self,dt,d):
    	""" Explicitly evaluate for signal derivatives.

    	Arguments:
    		dt - timestep to use for the calculation
    		d  - MNODeriv object to get explicit derivative values from

    	Returns: 
    		new MNODeriv object
    	"""
    	state = MNOState()

    	# update neuron 1
    	state.x1 = self.x[0] + (d.dx1 * dt)
    	state.v1 = self.v[0] + (d.dv1 * dt)
    	state.y1 = max(state.x1, 0.)

    	# update neuron 2
    	state.x2 = self.x[1] + (d.dx2 * dt)
    	state.v2 = self.v[1] + (d.dv2 * dt)
    	state.y2 = max(state.x2, 0.)

    	# Calculate the derivative
    	return self.__mno_solve(state)

    def __mno_solve(self,state):
    	""" Calculate the derivatives of the neural oscillator.

    	Arguments:
    		state - MNOState object holding the current state.

    	Returns:
    		calculated derivative placed in an MNODeriv object
    	"""
    	result = MNODeriv()

    	# neuron 1
    	result.dx1 = (-state.x1 + self.c - self.a * state.y2 - self.b * state.v1)/self.tau
    	result.dv1 = (-state.v1 + state.y1) / self.T

    	# neuron 2
    	result.dx2 = (-state.x2 + self.c - self.a * state.y1 - self.b * state.v2)/self.tau
    	result.dv2 = (-state.v2 + state.y2) / self.T

    	return result

    def __validate_params(self):
    	""" Ensure that the parameters are the correct values. """
    	while(self.a >= (1. + self.b)):
    		if(random.random() < 0.5):
    			self.a = random.random() * (self.b + 1.)
    		else:
    			self.b = (self.a-1.) + (random.random()*(self.limit-self.a))
    	while(self.tau == 0):
    		self.tau = random.random()*(self.a-1.)
    	while(self.T == 0):
    		self.T = random.random()*(self.a-1.)
    	while(1 + (self.tau/self.T) >= self.a):
    		if(random.random() < 0.5):
    			self.tau = random.random()*(self.a-1.)*self.T
    			while(self.tau == 0):
    				self.tau = random.random()*(self.a-1.)*self.T
    		else:
    			self.T = self.T+(random.random()*(self.T))
    			while(self.T == 0):
    				self.T = self.T+(random.random()*(self.T))

    def mutate(self):
    	""" Mutate the neural oscillator.

    	Note:
    		Stable system oscillation requires that the following equation must hold.
    		1 + (tau/T) < a < (1+b)
    	"""
    	choice = random.randrange(4)

    	if choice == 0:
    		self.a = (1.+self.tau/self.T) + ((1.+self.b)-(1.+(self.tau/self.T))*random.random())
        elif choice == 1:
    		self.b = (self.a-1.)+(self.limit-(self.a-1.))*random.random()
    	elif choice == 2:
    		self.tau = random.random() * (self.a-1.)*self.T
    	else:
    		self.T = random.random() * (self.a-1.)/self.tau
    	self.__validate_params()

class MNODeriv(object):
	""" Derivative values for MNO signals. """

	def __init__(self):
		""" Initialize the four derivative values to zero. """

		# neuron 1 signal derivatives
		self.dx1 = 0.
		self.dv1 = 0.

		# neuron 2 signal derivatives
		self.dx2 = 0.
		self.dv2 = 0.

class MNOState(object):
	""" State of the MNO (values of the signals) """

	def __init__(self):
		""" Initialize the signal values """

		# neuron 1 signals
		self.x1 = 0.
		self.v1 = 0.
		self.y1 = 0.

		# neuron 2 signals
		self.x2 = 0.
		self.v2 = 0.
		self.y1 = 0.
