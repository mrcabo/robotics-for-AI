

class RecognisedObject(object):

    def __init__(self, s):
        splitted = s.split(';')
        self.class_name = splitted[0]
        self.center_x = float(splitted[1])
        self.center_y = float(splitted[2])
        self.center_z = float(splitted[3])

    def __str__(self):
        return "{};{};{};{}".format(self.class_name, self.center_x, self.center_y, self.center_z)
