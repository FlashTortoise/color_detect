import precise_turning as trn
import car_find_color as car
import tortoise as t


class TestPatio2(t.Task):
    def __init__(self):
        super(TestPatio2, self).__init__()

        self.precise_turning = trn.Turning()
        self.car_find_color = car.ColorTracingTask()

    def step(self):
        if not self.car_find_color.turn_over:
            self.car_find_color.step()
        else:
            self.precise_turning.step()


if __name__ == '__main__':
    tttt = t.Tortoise()
    tttt.task = TestPatio2()
    tttt.walk()
