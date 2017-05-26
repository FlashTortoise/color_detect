from tortoise import p, Task, Tortoise


class PIDController(object):
    def __init__(self, kp, ki, kd):
        self._kp, self._ki, self._kd = kp, ki, kd

        self._a0 = self._kp + self._ki + self._kd
        self._a1 = (- self._kp) - 2 * self._kd
        self._a2 = self._kd

        self.lx = 0
        self.llx = 0
        self.ly = 0

    def run(self, x):
        y = self.ly + self._a0 * x + self._a1 * self.lx + self._a2 * self.llx
        self.ly, self.lx, self.llx = y, x, self.lx
        return y

    @property
    def kp(self):
        return self._kp

    @property
    def ki(self):
        return self._ki

    @property
    def kd(self):
        return self._kd

    @kp.setter
    def kp(self, value):
        self._kp = value
        self._a0 = self._kp + self._ki + self._kd
        self._a1 = (- self._kp) - 2 * self._kd

    @ki.setter
    def ki(self, value):
        self._ki = value
        self._a0 = self._kp + self._ki + self._kd

    @kd.setter
    def kd(self, value):
        self._kd = value
        self._a0 = self._kp + self._ki + self._kd
        self._a1 = (- self._kp) - 2 * self._kd
        self._a2 = self._kd


def constrain(a, l, u):
    if a > u:
        return u
    elif a < l:
        return l
    else:
        return a


class Turning(Task):
    def __init__(self):
        super(Turning, self).__init__()
        self.c = PIDController(0.04, 0, 0)
        self.target_yaw = 0

    def step(self):
        deg = p.gyroscope.get()
        diff = constrain(self.c.run(self.target_yaw - deg), -0.4, 0.4)

        print deg, diff
        p.wheels.set_diff(speed=0, diff=diff)

        return abs(self.target_yaw - deg) < 5


if __name__ == '__main__':
    tt = Tortoise()
    tt.task = Turning()
    tt.walk()
