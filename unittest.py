import unittest
import math
import itertools


from algorithms import eucDistance
from algorithms import manDistance
from algorithms import heuristic
from algorithms import metaheuristic
from algorithms import exact

def buildCoords(content, totalnodes):

    system = []

    i = 0
    while i < totalnodes:
        x = content[7+i].split()
        system.append(x)
        i += 1

    k = getDemand(content)

    k += 1

    i = 0
    while i < totalnodes:
        x=content[k+i].split()

        if len(system[i]) == 3:
            system[i].append(0)
        system[i][3] = x[1]
        i += 1
    return system


def calcRouteDist(route, system):

    totalDist = 0
    for i in range(0,len(route)):
        for j in range(0,len(route[i])-2):

            x1 = system[route[i][j]-1][1]
            y1 = system[route[i][j]-1][2]
            x2 = system[route[i][j+1]-1][1]
            y2 = system[route[i][j+1]-1][2]

            totalDist += eucDistance(x1,y1,x2,y2)

    return totalDist

def getDemand(content):
    i = 0
    k = len(content)
    while i < k:
        if content[i] == "DEMAND_SECTION":
            return  i
        i += 1



class eucDistanceTest(unittest.TestCase):
    def test(self):
        self.assertEqual(eucDistance(4,2,6,4), math.sqrt(8))

class manDistanceTest(unittest.TestCase):
    def test(self):
        self.assertEqual(manDistance(4,2,6,4), 4)

class calcRouteDistTest(unittest.TestCase):
    def test(self):
        system = [[1, 5, 5, 0], [2, 0, 6, 1], [3, 0, 4, 1], [4, 10, 6, 1], [5, 10, 4, 1], [6, 10, 5, 1]]
        route = [[1, 3, 2, 1, 2], [1, 5, 6, 4, 1, 3]]
        x = calcRouteDist(route,system)
        self.assertEqual(x, 24.396078054371138)


class buildcoordsTest(unittest.TestCase):
    def test(self):
        content = ['NAME : att48', 'COMMENT : (Rinaldi,Yarrow/Araque, Min no of trucks: 4, Best value: 12649)', 'TYPE : CVRP', 'DIMENSION : 6', 'EDGE_WEIGHT_TYPE : EUC_2D', 'CAPACITY : 3', 'NODE_COORD_SECTION', '1 5 5', '2 0 6', '3 0 4', '4 10 6', '5 10 4', '6 10 5', '', 'DEMAND_SECTION', '1 0', '2 1', '3 1', '4 1', '5 1', '6 1', '', 'DEPOT_SECTION', '1', '-1', 'EOF']

        a = buildCoords(content,6)

        b = [['1', '5', '5', '0'], ['2', '0', '6', '1'], ['3', '0', '4', '1'], ['4', '10', '6', '1'], ['5', '10', '4', '1'], ['6', '10', '5', '1']]

        self.assertEqual(a, b)

class heuristicTest(unittest.TestCase):
    def testNodesVisited(self):
        system = [[1,5,5,0],[2,0,6,1],[3,0,4,1],[4,10,6,1],[5,10,4,1],[6,10,5,1]]
        capacity = 3
        output = heuristic(system,capacity)

        for i in range(0,len(output)):
            del output[i][-1]
        a = list(itertools.chain.from_iterable(output))

        for i in range(len(system)-1):
            self.assertIn(i+1, a)


    def testStartEndatDepot(self):
        system = [[1, 5, 5, 0], [2, 0, 6, 1], [3, 0, 4, 1], [4, 10, 6, 1], [5, 10, 4, 1], [6, 10, 5, 1]]
        capacity = 3
        output = heuristic(system, capacity)

        for i in range(len(output)):
            self.assertEqual(output[i][0], 1)
            self.assertEqual(output[i][-2], 1)

    def testCapacityLimit(self):
        system = [[1, 5, 5, 0], [2, 0, 6, 1], [3, 0, 4, 1], [4, 10, 6, 1], [5, 10, 4, 1], [6, 10, 5, 1]]
        capacity = 3
        output = heuristic(system, capacity)

        for i in range(len(output)):
            demand = 0
            for j in range(len(output[i])-1):
                a = output[i][j]
                demand += system[a-1][-1]

        flag = False

        if demand<=capacity:
            flag = True
        self.assertEqual(flag,True)

class metaheuristicTest(unittest.TestCase):
    def testNodesVisited(self):
        system = [[1,5,5,0],[2,0,6,1],[3,0,4,1],[4,10,6,1],[5,10,4,1],[6,10,5,1]]
        capacity = 3

        print('Testing metaheuristic algorithm, please wait a few seconds')
        output = metaheuristic(system,capacity)
        print('Test complete')

        for i in range(0,len(output)):
            del output[i][-1]
        a = list(itertools.chain.from_iterable(output))

        for i in range(len(system)-1):
            self.assertIn(i+1, a)


    def testCapacityLimit(self):
        system = [[1, 5, 5, 0], [2, 0, 6, 1], [3, 0, 4, 1], [4, 10, 6, 1], [5, 10, 4, 1], [6, 10, 5, 1]]
        capacity = 3

        print('Testing metaheuristic algorithm, please wait a few seconds')
        output = metaheuristic(system, capacity)
        print('Test Complete')

        for i in range(len(output)):
            demand = 0
            for j in range(len(output[i])-1):
                a = output[i][j]
                demand += system[a-1][-1]

        flag = False

        if demand<=capacity:
            flag = True
        self.assertEqual(flag,True)

class exactTest(unittest.TestCase):
    def testNodesVisited(self):
        system = [[1,5,5,0],[2,0,6,1],[3,0,4,1],[4,10,6,1],[5,10,4,1],[6,10,5,1]]
        capacity = 3
        output = exact(system,capacity)

        for i in range(0,len(output)):
            del output[i][-1]
        a = list(itertools.chain.from_iterable(output))

        for i in range(len(system)-1):
            self.assertIn(i+1, a)


    def testStartEndatDepot(self):
        system = [[1, 5, 5, 0], [2, 0, 6, 1], [3, 0, 4, 1], [4, 10, 6, 1], [5, 10, 4, 1], [6, 10, 5, 1]]
        capacity = 3
        output = exact(system, capacity)

        for i in range(len(output)):
            self.assertEqual(output[i][0], 1)
            self.assertEqual(output[i][-2], 1)

    def testCapacityLimit(self):
        system = [[1, 5, 5, 0], [2, 0, 6, 1], [3, 0, 4, 1], [4, 10, 6, 1], [5, 10, 4, 1], [6, 10, 5, 1]]
        capacity = 3
        output = exact(system, capacity)

        for i in range(len(output)):
            demand = 0
            for j in range(len(output[i])-1):
                a = output[i][j]
                demand += system[a-1][-1]

        flag = False

        if demand<=capacity:
            flag = True
        self.assertEqual(flag,True)






def main():
    unittest.main()

if __name__ == '__main__':
    main()