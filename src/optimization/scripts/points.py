#! /usr/bin/env python3
# points=[(25.20, 25.52),(13.72, 20.53),(26.64, 13.37),(1.64, 24.88),(-5.45, 6.99),(7.91, -5.22),(18.17, 5.611),(24.31, -4.70)]
# starting=0

from tempfile import tempdir
import rospy
import math
from matplotlib import pyplot as plt
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class PointPublisher():
    def __init__(self) -> None:
        self.point_pub = rospy.Publisher(
            "/goal_sorted", Path, queue_size=1)

publisher = PointPublisher()

class OptimalPath():
    def __init__(self) -> None:
        self.finish = False
        self.points = list()
        self.starting = 0
        self.path_msg = Path()
        rospy.Subscriber("/goal_points", Path, self.callback, queue_size=1)



    def callback(self,msg):
        for point in msg.poses:
            temp_tuple = (point.pose.position.x,point.pose.position.y)
            self.points.append(temp_tuple)

    def create_data_model(self,points,starting):
        """Stores the data for the problem."""
        data = {}
        # Locations in block units
        data['locations'] = points
        data['num_vehicles'] = 1
        data['depot'] = starting

        return data

    def compute_euclidean_distance_matrix(self,locations):
        """Creates callback to return distance between points."""
        distances = {}
        for from_counter, from_node in enumerate(locations):
            distances[from_counter] = {}
            for to_counter, to_node in enumerate(locations):
                if from_counter == to_counter:
                    distances[from_counter][to_counter] = 0
                else:
                    # Euclidean distance
                    distances[from_counter][to_counter] = (int(
                        math.hypot((from_node[0] - to_node[0]),
                                (from_node[1] - to_node[1]))))
        return distances

    def print_solution(self,manager, routing, solution):
        """Prints solution on console."""
        index = routing.Start(0)
        plan_output = []
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output.append(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    
        return plan_output

    def compute(self,points,starting=0):
        """Entry point of the program."""
        # Instantiate the data problem.
        data = self.create_data_model(points,starting)

        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(len(data['locations']),
                                            data['num_vehicles'], data['depot'])

        # Create Routing Model.
        routing = pywrapcp.RoutingModel(manager)

        distance_matrix = self.compute_euclidean_distance_matrix(data['locations'])

        def distance_callback(from_index, to_index):
            """Returns the distance between the two nodes."""
            # Convert from routing variable Index to distance matrix NodeIndex.
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return distance_matrix[from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)

        # Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.GLOBAL_CHEAPEST_ARC)

        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)

        print('Objective: {}'.format(solution.ObjectiveValue()))

        # Print solution on console.
        if solution:
            return self.print_solution(manager, routing, solution)
    
    def permutation(self):
        if(not self.finish and len(self.points) != 0):
            
            permutation = self.compute(self.points,starting=self.starting)

            for i in permutation:
                temp_var = PoseStamped()
                temp_var.header.frame_id = "velodyne"
                temp_var.pose.position.x = self.points[i][0]
                temp_var.pose.position.y = self.points[i][1]
                self.path_msg.poses.append(temp_var)

            self.path_msg.header.frame_id = "velodyne"
            publisher.point_pub.publish(self.path_msg)


            #For Visualization. Comment on run.
            points2 = list()
            for i in permutation:
                points2.append(self.points[i])
            x,y = map(list,zip(*points2))
            print(x)
            print(y)
            plt.plot(x,y,'-o')
            # plt.show()
            self.finish = True
        elif(self.finish and len(self.points) != 0):
            publisher.point_pub.publish(self.path_msg)





def main():
    
    rospy.init_node("location_point_pub", anonymous=False)
    optimal_point = OptimalPath()

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        optimal_point.permutation()


if __name__=='__main__':
    main()
