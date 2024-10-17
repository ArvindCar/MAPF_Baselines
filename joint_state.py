import time as timer
from single_agent_planner import compute_heuristics, joint_state_a_star, get_sum_of_cost


class JointStateSolver(object):
    """A planner that plans for all robots together."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []

        ##############################
        # Task 1.1: Read function prototype and call joint_state_a_star here
        #  
        path = joint_state_a_star(self.my_map, self.starts, self.goals, self.heuristics, self.num_of_agents) 

        if path is None:
            return None
                
        # Task 1.1: Convert the joint state path to a list of paths for each agent
        #  
        
        ##############################
        for j in range(self.num_of_agents):
            agent_path = []
            for step in range(len(path)):
                agent_path.append((path[step][j][0], path[step][j][1]))
            goal = agent_path[-1]
            for step in range(len(agent_path)-2,-1,-1):
                if goal == agent_path[step]:
                    agent_path.pop(step)
                else:
                    break
            result.append(agent_path)
        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
