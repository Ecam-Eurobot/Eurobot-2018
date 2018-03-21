class Points:

    def __init__(self):

        # Points
        self.points = {}
        self.points_max = {}
        self.state = {}

        # Water
        self.points['recuperator_emptied'] = 0
        self.points_max['recuperator_emptied'] = 10 * 2
        self.state['recuperator'] = [False, False]

        self.points['watertower'] = 0
        self.points_max['watertower'] = (8 + 4) * 5
        self.state['watertower_balls_remaining'] = 8 + 4

        self.points['treatment_plant'] = 0
        self.points_max['treatment_plant'] = 4 * 5


        # Towers
        self.points['tower'] = 0
        self.points_max['tower'] = (1 + 2 + 3 + 4 + 5 + 30) * 3
        self.state['towers_remaining'] = 3

        # Panel
        self.points['panel_placement'] = 5
        self.points_max['panel_placement'] = 5

        self.points['panel_power'] = 0
        self.points_max['panel_power'] = 25

        # Bee
        self.points['bee_placement'] = 5
        self.points_max['bee_placement'] = 5

        self.points['bee_popped'] = 0
        self.points_max['bee_popped'] = 50


    def total_points(self):
        return sum(self.points.values())

    def max_total_points(self):
        return sum(self.points_max.values())


    def emptied_recuperator(self, n):
        """Signals that the ball recuperator n is emptied and the points
        are added to the total if it was not already emptied"""

        r = self.state['recuperator']

        if n < 0 or n >= len(r):
            raise ValueError("n must be between 0 and " + str(len(r)-1))

        if not r[n]:
            self.points['recuperator_emptied'] += 10
            self.state['recuperator'][n] = True
        else:
            raise ValueError("Recuperator " + str(n) + " is already emptied")


    def balls_shot_in_watertower(self, n):
        """Signals that n balls where shot into the watertower"""

        b = self.state['watertower_balls_remaining']

        if n > b:
            ValueError("n is greater than the number of balls remaining: " + str(b))

        self.points['watertower'] += 5 * n
        self.state['watertower_balls_remaining'] -= n

    def balls_in_treatment_plant(self):
        """Signals that the balls were put in the water treatment plant"""

        if self.points['treatment_plant'] < self.points_max['treatment_plant']:
            self.points['treatment_plant'] = self.points_max['treatment_plant']
        else:
            raise ValueError("Balls are already in the treatment plant")


    def placed_tower(self, height, sequence=False):
        """Signals that a tower is built and adds the points"""

        if self.state['towers_remaining'] <= 0:
            ValueError("All the towers are already built")

        self.points['tower'] += (height * (height + 1)) / 2 + int(sequence) * 30
        self.state['towers_remaining'] -= 1


    def panel_powered(self):
        """Signals that the panel is powered up and adds the points"""
        self.points['panel_power'] = self.points_max['panel_power']


    def bee_popped(self):
        """Signals that the bee popped the balloon and adds the points"""
        self.points['bee_popped'] = self.points_max['bee_popped']


if __name__ == '__main__':
    points = Points()

    print("Points: " + str(points.total_points()) + "/" + str(points.max_total_points()))
    points.emptied_recuperator(0)
    print("Points: " + str(points.total_points()) + "/" + str(points.max_total_points()))
    points.balls_shot_in_watertower(8)
    print("Points: " + str(points.total_points()) + "/" + str(points.max_total_points()))
    points.balls_in_treatment_plant()
    print("Points: " + str(points.total_points()) + "/" + str(points.max_total_points()))
    points.placed_tower(5, sequence=True)
    print("Points: " + str(points.total_points()) + "/" + str(points.max_total_points()))
    points.placed_tower(5)
    print("Points: " + str(points.total_points()) + "/" + str(points.max_total_points()))



