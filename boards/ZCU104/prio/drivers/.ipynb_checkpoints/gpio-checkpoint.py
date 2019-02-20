
class GPIO():	
	DATA_OFFSET = 	0x00
	TRI_OFFSET =	0x04
	
	ALL_INPUTS = 	0xFFFF
	ALL_OUTPUTS = 	0x0000

	def __init__(self, partial_region):
		self._partial_region = partial_region

	def setDirection(self, direction):
		self._partial_region.write(self.TRI_OFFSET, direction)

	def getDirection(self):
		return self._partial_region.read(self.TRI_OFFSET)

	def setValue(self, value):
		self._partial_region.write(self.DATA_OFFSET, value)

	def getValue(self):
		return self._partial_region.read(self.DATA_OFFSET)


