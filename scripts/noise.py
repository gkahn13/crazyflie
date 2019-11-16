import numpy as np


class Noise(object):

    def __init__(self, **kwargs):
        self.reset()

    def step(self):
        raise NotImplementedError

    def reset(self):
        raise NotImplementedError


class UniformNoiseEveryStep(Noise):

    def __init__(self, **kwargs):
        self._range = kwargs['range']

        super(UniformNoiseEveryStep, self).__init__(**kwargs)

    def step(self):
        return np.random.uniform(*self._range)

    def reset(self):
        pass


class UniformNoiseEveryReset(Noise):

    def __init__(self, **kwargs):
        self._range = kwargs['range']
        self._step_range = kwargs['step_range']

        self._noise = None

        super(UniformNoiseEveryReset, self).__init__(**kwargs)

    def step(self):
        step_noise = self._noise + np.random.uniform(*self._step_range)
        step_noise = np.clip(step_noise, *self._range)
        return step_noise

    def reset(self):
        self._noise = np.random.uniform(*self._range)


class OUNoise(Noise):

    def __init__(self, **kwargs):
        self._mu = kwargs['mu']
        self._theta = kwargs['theta']
        self._sigma = kwargs['sigma']
        self._range = kwargs['range']

        self._noise = None

        super(OUNoise, self).__init__(**kwargs)

    def step(self):
        prev_noise_normalized = self._noise / max(self._range)
        noise_normalized = prev_noise_normalized + \
                           self._theta * (self._mu - prev_noise_normalized) + \
                           self._sigma * np.random.randn(1)
        self._noise = max(self._range) * noise_normalized
        return self._noise

    def reset(self):
        self._noise = np.random.uniform(*self._range)


class StepNoise(Noise):

    def __init__(self, **kwargs):
        self._reset_step_range = kwargs['reset_step_range']
        self._step_range = kwargs['step_range']
        self._range = kwargs['range']

        self._curr_mean = None
        self._curr_reset_step = None
        self._t = 0
        super(StepNoise, self).__init__(**kwargs)

    def step(self):
        if self._t >= self._curr_reset_step:
            self.reset()

        noise = self._curr_mean + np.random.uniform(*self._step_range)
        noise = np.clip(noise, *self._range)
        self._t += 1

        return noise

    def reset(self):
        self._t = 0
        self._curr_mean = np.random.uniform(*self._range)
        self._curr_reset_step = np.random.randint(*self._reset_step_range)