# assignment-4

1st problem:-


# Python code to give the traversed path by BFS algorithm as output. BFS(int n) scans through the vertices which are reachable from n.  
from collections import defaultdict as dd  
  
class Graph:  
    
    # Constructing a list  
    def __init__(self):  
    
        # default dictionary to store graph  
        self.graph = dd(list)  
    
    # defining function which will add edge to the graph  
    def addEdgetoGraph(self, x, y):  
        self.graph[x].append(y)  
    
    # defining function to print BFS traverse  
    def BFSearch(self, n):  
    
        # Initially marking all vertices as not visited  
        visited_vertices = ( len(self.graph ))*[False]  
    
        # creating a queue for visited vertices  
        queue = []  
    
        # setting source node as visited and adding it to the queue  
        visited_vertices[n] = True  
        queue.append(n)  
          
    
        while queue:  
    
            # popping the element from the queue which is printed  
            n = queue.pop(0)  
            print (n, end = " ")  
    
            # getting vertices adjacent to the vertex n which is dequed.   
            for v in self.graph[ n ]:  
                if visited_vertices[v] == False:  
                    queue.append(v)  
                    visited_vertices[v] = True  
    
  
# Example code  
# Initializing a graph  
graph = Graph()  
graph.addEdgetoGraph(0, 1)  
graph.addEdgetoGraph(1, 1)  
graph.addEdgetoGraph(2, 2)  
graph.addEdgetoGraph(3, 1)  
graph.addEdgetoGraph(4, 3)  
graph.addEdgetoGraph(5, 4)  
    
print ( " The Breadth First Search Traversal for The Graph is as Follows: " )  
graph.BFSearch(3)  

2nd problem:-

# Python program to print DFS traversal for complete graph
from collections import defaultdict

# This class represents a directed graph using adjacency
# list representation
class Graph:

	# Constructor
	def __init__(self):

		# default dictionary to store graph
		self.graph = defaultdict(list)

	# function to add an edge to graph
	def addEdge(self,u,v):
		self.graph[u].append(v)

	# A function used by DFS
	def DFSUtil(self, v, visited):

		# Mark the current node as visited and print it
		visited[v]= True
		print v,

		# Recur for all the vertices adjacent to
		# this vertex
		for i in self.graph[v]:
			if visited[i] == False:
				self.DFSUtil(i, visited)


	# The function to do DFS traversal. It uses
	# recursive DFSUtil()
	def DFS(self):
		V = len(self.graph) #total vertices

		# Mark all the vertices as not visited
		visited =[False]*(V)

		# Call the recursive helper function to print
		# DFS traversal starting from all vertices one
		# by one
		for i in range(V):
			if visited[i] == False:
				self.DFSUtil(i, visited)


# Driver code
# Create a graph given in the above diagram
g = Graph()
g.addEdge(0, 1)
g.addEdge(0, 2)
g.addEdge(1, 2)
g.addEdge(2, 0)
g.addEdge(2, 3)
g.addEdge(3, 3)

print "Following is Depth First Traversal"
g.DFS()

3rd problem:-

# Python3 program to print
# count of nodes at given level.
from collections import deque

adj = [[] for i in range(1001)]

def addEdge(v, w):
	
	# Add w to v’s list.
	adj[v].append(w)

	# Add v to w's list.
	adj[w].append(v)

def BFS(s, l):
	
	V = 100
	
	# Mark all the vertices
	# as not visited
	visited = [False] * V
	level = [0] * V

	for i in range(V):
		visited[i] = False
		level[i] = 0

	# Create a queue for BFS
	queue = deque()

	# Mark the current node as
	# visited and enqueue it
	visited[s] = True
	queue.append(s)
	level[s] = 0

	while (len(queue) > 0):
		
		# Dequeue a vertex from
		# queue and print
		s = queue.popleft()
		#queue.pop_front()

		# Get all adjacent vertices
		# of the dequeued vertex s.
		# If a adjacent has not been
		# visited, then mark it
		# visited and enqueue it
		for i in adj[s]:
			if (not visited[i]):

				# Setting the level
				# of each node with
				# an increment in the
				# level of parent node
				level[i] = level[s] + 1
				visited[i] = True
				queue.append(i)

	count = 0
	for i in range(V):
		if (level[i] == l):
			count += 1
			
	return count

# Driver code
if __name__ == '__main__':
	
	# Create a graph given
	# in the above diagram
	addEdge(0, 1)
	addEdge(0, 2)
	addEdge(1, 3)
	addEdge(2, 4)
	addEdge(2, 5)

	level = 2

	print(BFS(0, level))

 
	4th problem:-

 # Python3 program to count number
# of trees in a forest.

# A utility function to add an
# edge in an undirected graph.
def addEdge(adj, u, v):
	adj[u].append(v)
	adj[v].append(u)

# A utility function to do DFS of graph
# recursively from a given vertex u.
def DFSUtil(u, adj, visited):
	visited[u] = True
	for i in range(len(adj[u])):
		if (visited[adj[u][i]] == False):
			DFSUtil(adj[u][i], adj, visited)

# Returns count of tree is the
# forest given as adjacency list.
def countTrees(adj, V):
	visited = [False] * V
	res = 0
	for u in range(V):
		if (visited[u] == False):
			DFSUtil(u, adj, visited)
			res += 1
	return res

# Driver code
if __name__ == '__main__':

	V = 5
	adj = [[] for i in range(V)]
	addEdge(adj, 0, 1)
	addEdge(adj, 0, 2)
	addEdge(adj, 3, 4)
	print(countTrees(adj, V))


5th problem:-

# Python program to detect cycle
# in a graph

from collections import defaultdict

class Graph():
	def __init__(self, vertices):
		self.graph = defaultdict(list)
		self.V = vertices

	def addEdge(self, u, v):
		self.graph[u].append(v)

	def isCyclicUtil(self, v, visited, recStack):

		# Mark current node as visited and
		# adds to recursion stack
		visited[v] = True
		recStack[v] = True

		# Recur for all neighbours
		# if any neighbour is visited and in
		# recStack then graph is cyclic
		for neighbour in self.graph[v]:
			if visited[neighbour] == False:
				if self.isCyclicUtil(neighbour, visited, recStack) == True:
					return True
			elif recStack[neighbour] == True:
				return True

		# The node needs to be popped from
		# recursion stack before function ends
		recStack[v] = False
		return False

	# Returns true if graph is cyclic else false
	def isCyclic(self):
		visited = [False] * self.V
		recStack = [False] * self.V
		for node in range(self.V):
			if visited[node] == False:
				if self.isCyclicUtil(node, visited, recStack) == True:
					return True
		return False

g = Graph(4)
g.addEdge(0, 1)
g.addEdge(0, 2)
g.addEdge(1, 2)
g.addEdge(2, 0)
g.addEdge(2, 3)
g.addEdge(3, 3)
if g.isCyclic() == 1:
	print "Graph has a cycle"
else:
	print "Graph has no cycle"

n-queen's problem:-

# Python program to solve N Queen
# Problem using backtracking

global N
N = 4

def printSolution(board):
	for i in range(N):
		for j in range(N):
			print (board[i][j],end=' ')
		print()


# A utility function to check if a queen can
# be placed on board[row][col]. Note that this
# function is called when "col" queens are
# already placed in columns from 0 to col -1.
# So we need to check only left side for
# attacking queens
def isSafe(board, row, col):

	# Check this row on left side
	for i in range(col):
		if board[row][i] == 1:
			return False

	# Check upper diagonal on left side
	for i, j in zip(range(row, -1, -1), range(col, -1, -1)):
		if board[i][j] == 1:
			return False

	# Check lower diagonal on left side
	for i, j in zip(range(row, N, 1), range(col, -1, -1)):
		if board[i][j] == 1:
			return False

	return True

def solveNQUtil(board, col):
	# base case: If all queens are placed
	# then return true
	if col >= N:
		return True

	# Consider this column and try placing
	# this queen in all rows one by one
	for i in range(N):

		if isSafe(board, i, col):
			# Place this queen in board[i][col]
			board[i][col] = 1

			# recur to place rest of the queens
			if solveNQUtil(board, col + 1) == True:
				return True

			# If placing queen in board[i][col
			# doesn't lead to a solution, then
			# queen from board[i][col]
			board[i][col] = 0

	# if the queen can not be placed in any row in
	# this column col then return false
	return False

# This function solves the N Queen problem using
# Backtracking. It mainly uses solveNQUtil() to
# solve the problem. It returns false if queens
# cannot be placed, otherwise return true and
# placement of queens in the form of 1s.
# note that there may be more than one
# solutions, this function prints one of the
# feasible solutions.
def solveNQ():
	board = [ [0, 0, 0, 0],
			[0, 0, 0, 0],
			[0, 0, 0, 0],
			[0, 0, 0, 0]
			]

	if solveNQUtil(board, 0) == False:
		print ("Solution does not exist")
		return False

	printSolution(board)
	return True

# driver program to test above function
solveNQ()




 

