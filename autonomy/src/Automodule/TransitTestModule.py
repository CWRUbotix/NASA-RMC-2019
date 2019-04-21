
import sys
#import rospy
from PathPlanning.PathPlanning import Grid, Obstacle, Position
from PathPlanning.ThetaStar import create_path
from PathPlanning.PathTesting import drawPath
#from TestModule import turn_algo_2, conservative_drive, testShutdown

ARENA_WIDTH = 3.78
ARENA_HEIGHT = 7.38
ROBOT_WIDTH = 0.75
ROBOT_LENGTH = 1.5

def rosTest():
    pass

def softwareTest():
    print 'checking pathplanning stuff'
    p1 = Position(1.0, 1.0)
    p2 = Position(3.0, 7.0)
    grid = Grid(p1, p2, 3.78, 7.38)
    print str(grid.unit_width)
    print str(grid.unit_height)
    print str(grid.col_size)
    print str(grid.row_size)
    obstacles = [Obstacle(2.0, 2.0, 0.15), Obstacle(1.0, 3.0, 0.15)]

    path = create_path(p1, p2, 3.78, 7.38, obstacles)
    drawPath(path, obstacles, [])

def main():
    if sys.argv[1] == '0':
        rosTest()
    elif sys.argv[1] == '1':
        softwareTest()
    else:
        print 'unsupported'
        exit(0)

if __name__ == '__main__':
    main()