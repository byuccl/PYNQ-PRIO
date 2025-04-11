from pynq.overlay import Overlay
from pynq import GPIO

class PrIoOverlay(Overlay):
	PACKAGE_ROOT="/usr/local/share/pynq-venv/lib/python3.10/site-packages/pynq/overlays/"
	FULL_BITSTREAM_PATH = PACKAGE_ROOT + "prio_linux/"
	PARTIAL_BITSTREAM_PATH = PACKAGE_ROOT + "prio_linux/partial/"
	DTBO_PATH = PACKAGE_ROOT + "prio_linux/dtbo/"

	def __init__(self, bitfile_name, dtbo=None, download=True, ignore_version=False):
		super().__init__(bitfile_name)

	def pr_download(self, partial_region, partial_bit, dtbo_path=None):
		decoupler = GPIO(int(partial_region[3]) + GPIO.get_gpio_base(), 'out')
		decoupler.write(1)
		super().pr_download(partial_region, partial_bit, dtbo_path)
		decoupler.write(0)
