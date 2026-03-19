#!/usr/bin/python3
import shutil

LOGGING_NAME = 'DiskUtils'


def bytes_to_gb(bytes_value):
    """Converts bytes to gigabytes (GiB) for human readability."""
    # Using 1024 for GiB (standard for disk space)
    return round(bytes_value / (1024**3), 2)


class DiskUtils:
    def __init__(self, path='/'):
        self.path = path
        self._total_disk_space = 0.0  # GB
        self._used_disk_space = 0.0  # GB
        self._free_disk_space = 0.0  # GB

    def get_total_disk_space(self):
        self._total_disk_space = bytes_to_gb(shutil.disk_usage(self.path).total)
        return self._total_disk_space

    def get_used_disk_space(self):
        self._used_disk_space = bytes_to_gb(shutil.disk_usage(self.path).used)
        return self._used_disk_space

    def get_free_disk_space(self):
        self._free_disk_space = bytes_to_gb(shutil.disk_usage(self.path).free)
        return self._free_disk_space
