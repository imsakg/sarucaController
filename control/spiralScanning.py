class spiralMovements(object):
    def __init__(self):
        self.MOVEMENTS = [[1, 1], [0, -1], [0, -1], [1, 1], [0, 1], [0, -1]]
        self.I = 0

    def __next__(self):
        movement = self.MOVEMENTS[self.I]
        self.I = self.I + 1 if (self.I + 1 < len(self.MOVEMENTS)) else 0
        return movement

    def __iter__(self):
        for i in self.MOVEMENTS:
            yield i

    def generate(self, m):
        self.MOVEMENTS = [
            [1 * m, 1 * m],
            [0 * m, -1 * m],
            [0 * m, -1 * m],
            [1 * m, 1 * m],
            [0 * m, 1 * m],
            [0 * m, -1 * m],
        ]
