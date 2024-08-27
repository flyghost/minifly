import subprocess
import threading

# Define the paths and commands
UV = r"C:\Keil_v5\UV4\UV4.exe"
UV_PRO_PATH = "./USER/Firmware_F411.uvprojx"

print("Init building ...")

def read_stream(stream):
    """Reads and prints lines from a stream."""
    try:
        while True:
            line = stream.readline()
            if not line:
                break
            print(line.strip())
    except Exception as e:
        print(f"Error reading stream: {e}")
    finally:
        stream.close()

# Method 2: Execute the compilation command and display the output in the terminal
try:
    # Run the command
    process = subprocess.Popen([UV, "-j0", "-r", UV_PRO_PATH], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)

    # Start a thread to read the output stream
    output_thread = threading.Thread(target=read_stream, args=(process.stdout,))
    output_thread.start()

    # Wait for the process to complete
    process.wait()

    # Ensure the thread has finished
    output_thread.join()
except Exception as e:
    print(f"An error occurred: {e}")

print("Done.")