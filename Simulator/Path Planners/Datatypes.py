import heapq
class Queue:
	def __init__(self):
		self.items=[]
	def isEmpty(self):
		return len(self.items)==0
	def length(self):
		return len(self.items)
	def put(self,a):
		return self.items.insert(0,a)
	def get(self):
		return self.items.pop()
	def _print(self):
		print(self.items)


class Stack:
	def __init__(self):
		self.items=[]
	def isEmpty(self):
		return len(self.items)==0
	def length(self):
		return len(self.items)
	def push(self,a):
		return self.items.append(a)
	def remove(self):
		return self.items.pop()
	def _print(self):
		print(self.items)
	def top(self):
		return self.items[len(self.items)-1]


class PriorityQueue:
	def __init__(self):
		self.elements = {}

	def isEmpty(self):
		return len(self.elements) == 0

	def put(self, item, priority):
		self.elements[item] = priority

	def get(self):
		best_item, best_priority = None, None
		for item, priority in self.elements.items():
			if best_priority is None or priority < best_priority:
				best_item, best_priority = item, priority

		del self.elements[best_item]
		return best_item
