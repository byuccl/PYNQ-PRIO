from pynq.overlay import Overlay


class PrIoOverlay(Overlay):

	def __init__(self, bitfile_name):
		super().__init__(bitfile_name)

	def download(self, partial_bit=None):
		super().download(partial_bit)
