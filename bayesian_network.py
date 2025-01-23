""" Bayesian networks """

from probability import BayesNet, enumeration_ask, elimination_ask, rejection_sampling, likelihood_weighting, gibbs_ask
from timeit import timeit, repeat
import pickle
import numpy as np

T, F = True, False

class DataPoint:
    """
    Represents a single datapoint gathered from one lap.
    Attributes are exactly the same as described in the project spec.
    """
    def __init__(self, muchfaster, early, overtake, crash, win):
        self.muchfaster = muchfaster
        self.early = early
        self.overtake = overtake
        self.crash = crash
        self.win = win

def generate_bayesnet():
    """
    Generates a BayesNet object representing the Bayesian network in Part 2
    returns the BayesNet object
    """
    bayes_net = BayesNet()
    # load the dataset, a list of DataPoint objects
    data = pickle.load(open("data/bn_data.p","rb"))
    # BEGIN_YOUR_CODE ######################################################

    T, F = True, False
    # function to calculate probabilities
    def calculate_probabilities(data, variable, parent_conditions=None):
        counts = {}
        total_counts = {}

        for d in data:
            parent_values = tuple(getattr(d, parent.lower()) for parent in parent_conditions) if parent_conditions else ()
            var_value = getattr(d, variable.lower())

            total_counts[parent_values] = total_counts.get(parent_values, 0) + 1
            if (parent_values, var_value) not in counts:
                counts[(parent_values, var_value)] = 0
            counts[(parent_values, var_value)] += 1

        probabilities = {}
        for (parent_values, var_value), count in counts.items():
            probabilities[(parent_values, var_value)] = count / total_counts[parent_values]

        return probabilities

    P_MuchFaster = calculate_probabilities(data, "MuchFaster")
    P_Early = calculate_probabilities(data, "Early")
    P_Overtake = calculate_probabilities(data, "Overtake", ["MuchFaster", "Early"])
    P_Crash = calculate_probabilities(data, "Crash", ["MuchFaster", "Early"])
    P_Win = calculate_probabilities(data, "Win", ["Overtake", "Crash"])

    P_MuchFaster_T = P_MuchFaster[((), True)]
    # print("mine: much faster", P_MuchFaster_T)

    P_Early_T = P_Early[((), True)] 
    # print("mine: early", P_Early_T)

    P_Overtake_values = {
        (T, T): P_Overtake[((True, True), True)],
        (T, F): P_Overtake[((True, False), True)],
        (F, T): P_Overtake[((False, True), True)],
        (F, F): P_Overtake[((False, False), True)],
    }
    # print("mine: overtake", P_Overtake_values)

    P_Crash_values = {
        (T, T): P_Crash[((True, True), True)],
        (T, F): P_Crash[((True, False), True)],
        (F, T): P_Crash[((False, True), True)],
        (F, F): P_Crash[((False, False), True)],
    }

    P_Win_values = {
        (T, T): P_Win[((True, True), True)],
        (T, F): P_Win[((True, False), True)],
        (F, T): P_Win[((False, True), True)],
        (F, F): P_Win[((False, False), True)],
    }
    # print("mine: crash", P_Crash_values)
    # print("mine: win", P_Win_values)

    bayes_net = BayesNet([
        ("MuchFaster", "", P_MuchFaster_T),
        ("Early", "", P_Early_T),
        ("Overtake", "MuchFaster Early", P_Overtake_values),
        ("Crash", "MuchFaster Early", P_Crash_values),
        ("Win", "Overtake Crash", P_Win_values),
    ])
    
    # END_YOUR_CODE ########################################################
    return bayes_net

def find_best_overtake_condition(bayes_net):
    """
    Finds the optimal condition for overtaking the car, as described in Part 3
    Returns the optimal values for (MuchFaster,Early)
    """
    # BEGIN_YOUR_CODE ######################################################
    T, F = True, False
    conditions = [(T, T), (T, F), (F, T), (F, F)]
    best_condition = (F, F)
    max_probability = 0

    for much_faster, early in conditions:
        # P(Win=True |MuchFaster=much_faster, Early=early)
        query_result = elimination_ask(
            'Win',
            {'MuchFaster': much_faster, 'Early': early, 'Crash': False},
            bayes_net
        )
        win_probability = query_result[T]  

        if win_probability > max_probability:
            max_probability = win_probability
            best_condition = (much_faster, early)

    print(f"Highest probability of winning: {max_probability}")
    return best_condition
    
    # END_YOUR_CODE ########################################################

def main():
    bayes_net = generate_bayesnet()
    cond = find_best_overtake_condition(bayes_net)
    print("Best overtaking condition: MuchFaster={}, Early={}".format(cond[0],cond[1]))

if __name__ == "__main__":
    main()

