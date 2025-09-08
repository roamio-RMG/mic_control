#!/usr/bin/env python3
"""
ReSpeaker USB Fix Script
This script fixes common USB permission and access issues with the ReSpeaker device.
Run this script whenever you encounter USB errors with the ReSpeaker.
"""

import subprocess
import os
import sys
import time

def run_command(command, description):
    """Run a command and return success status"""
    print(f"\n🔧 {description}...")
    try:
        result = subprocess.run(command, shell=True, check=True, text=True, capture_output=True)
        if result.stdout.strip():
            print(f"✅ {result.stdout}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ Error: {e}")
        if e.stderr:
            print(f"   Details: {e.stderr}")
        return False

def check_respeaker_device():
    """Check if ReSpeaker device is detected"""
    print("\n🔍 Checking for ReSpeaker device...")
    try:
        result = subprocess.run("lsusb | grep -i respeaker", shell=True, capture_output=True, text=True)
        if result.returncode == 0:
            print(f"✅ Found ReSpeaker: {result.stdout.strip()}")
            # Extract bus and device numbers
            parts = result.stdout.split()
            bus = parts[1]
            device = parts[3].rstrip(':')
            return bus, device
        else:
            print("❌ ReSpeaker device not found!")
            return None, None
    except Exception as e:
        print(f"❌ Error checking device: {e}")
        return None, None

def create_udev_rules():
    """Create comprehensive udev rules for ReSpeaker"""
    udev_content = """# ReSpeaker 4 Mic Array USB Permissions
# This rule ensures proper permissions for ReSpeaker USB device access

# Main USB device rule
SUBSYSTEM=="usb", ATTR{idVendor}=="2886", ATTR{idProduct}=="0018", MODE="0666", GROUP="plugdev"

# HID raw device rule
KERNEL=="hidraw*", ATTRS{idVendor}=="2886", ATTRS{idProduct}=="0018", MODE="0666", GROUP="plugdev"

# Additional USB subsystem rules
SUBSYSTEM=="usb", ATTRS{idVendor}=="2886", ATTRS{idProduct}=="0018", MODE="0666"
SUBSYSTEM=="usb_device", ATTRS{idVendor}=="2886", ATTRS{idProduct}=="0018", MODE="0666"

# Audio device rules (if applicable)
SUBSYSTEM=="sound", ATTRS{idVendor}=="2886", ATTRS{idProduct}=="0018", MODE="0666", GROUP="audio"
"""
    
    rule_file = "/etc/udev/rules.d/99-respeaker-fix.rules"
    print(f"\n📝 Creating udev rules at {rule_file}...")
    
    try:
        with open("/tmp/99-respeaker-fix.rules", "w") as f:
            f.write(udev_content)
        
        # Copy to system location with sudo
        cmd = f"sudo cp /tmp/99-respeaker-fix.rules {rule_file}"
        return run_command(cmd, "Installing udev rules")
    except Exception as e:
        print(f"❌ Error creating udev rules: {e}")
        return False

def add_user_to_groups():
    """Add current user to necessary groups"""
    groups = ["plugdev", "audio", "dialout"]
    username = os.getenv("USER")
    
    for group in groups:
        cmd = f"sudo usermod -a -G {group} {username}"
        run_command(cmd, f"Adding user to {group} group")

def reset_usb_device(bus, device):
    """Reset the specific USB device"""
    if bus and device:
        # Method 1: Using usb_modeswitch
        cmd1 = f"sudo usb_modeswitch -v 2886 -p 0018 --reset-usb"
        if run_command(cmd1, "Resetting USB device with usb_modeswitch"):
            return True
        
        # Method 2: Using USB device path
        device_path = f"/dev/bus/usb/{bus}/{device}"
        if os.path.exists(device_path):
            cmd2 = f"sudo chmod 666 {device_path}"
            return run_command(cmd2, f"Setting permissions on {device_path}")
    
    return False

def reload_udev_rules():
    """Reload udev rules and trigger device events"""
    commands = [
        ("sudo udevadm control --reload-rules", "Reloading udev rules"),
        ("sudo udevadm trigger", "Triggering udev events"),
        ("sudo systemctl restart systemd-udevd", "Restarting udev service")
    ]
    
    success = True
    for cmd, desc in commands:
        if not run_command(cmd, desc):
            success = False
    
    return success

def check_python_usb_libraries():
    """Check if required Python USB libraries are installed"""
    print("\n🐍 Checking Python USB libraries...")
    
    try:
        import usb.core
        import usb.backend.libusb1
        print("✅ Python USB libraries are installed")
        return True
    except ImportError as e:
        print(f"❌ Missing Python USB library: {e}")
        print("💡 Try installing with: pip install pyusb")
        return False

def main():
    print("🎤 ReSpeaker USB Fix Script")
    print("=" * 50)
    
    # Check if running as root
    if os.geteuid() == 0:
        print("⚠️  Warning: Don't run this script as root!")
        print("   Run as normal user - it will ask for sudo when needed.")
        sys.exit(1)
    
    # Step 1: Check for ReSpeaker device
    bus, device = check_respeaker_device()
    if not bus:
        print("\n❌ ReSpeaker device not found. Please:")
        print("   1. Check USB connection")
        print("   2. Try a different USB port")
        print("   3. Restart the device")
        sys.exit(1)
    
    # Step 2: Check Python libraries
    check_python_usb_libraries()
    
    # Step 3: Create udev rules
    if not create_udev_rules():
        print("❌ Failed to create udev rules")
        sys.exit(1)
    
    # Step 4: Add user to groups
    add_user_to_groups()
    
    # Step 5: Reload udev rules
    if not reload_udev_rules():
        print("❌ Failed to reload udev rules")
        sys.exit(1)
    
    # Step 6: Reset USB device
    reset_usb_device(bus, device)
    
    # Step 7: Final check
    print("\n🔄 Waiting for device to stabilize...")
    time.sleep(3)
    
    new_bus, new_device = check_respeaker_device()
    if new_bus:
        print("\n✅ ReSpeaker USB fix completed successfully!")
        print("\n📋 Next steps:")
        print("   1. Log out and log back in (or restart)")
        print("   2. Try running your ROS launch file again")
        print("   3. If issues persist, run this script again")
        
        # Check current permissions
        device_path = f"/dev/bus/usb/{new_bus}/{new_device}"
        if os.path.exists(device_path):
            result = subprocess.run(f"ls -la {device_path}", shell=True, capture_output=True, text=True)
            print(f"\n📁 Current device permissions: {result.stdout.strip()}")
    else:
        print("❌ Device not found after fix attempt")
        sys.exit(1)

if __name__ == "__main__":
    main()
