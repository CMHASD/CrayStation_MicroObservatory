import subprocess
import os

def create_ramdisk(mount_point="/mnt/ramdisk", size_mb=1):
    try:
        # Create the mount point if it doesn't exist
        if not os.path.exists(mount_point):
            subprocess.run(["sudo", "mkdir", "-p", mount_point], check=True)

        # Mount the RAM disk
        subprocess.run([
            "sudo", "mount", "-t", "tmpfs", "-o", f"size={size_mb}M", "tmpfs", mount_point
        ], check=True)

        print(f"RAM disk mounted at {mount_point} with size {size_mb}MB")
    except subprocess.CalledProcessError as e:
        print(f"Error creating RAM disk: {e}")

# ------------------------------------------------------------------------------------------
create_ramdisk()
create_ramdisk()
