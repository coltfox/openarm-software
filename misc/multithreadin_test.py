





import threading
import time
import queue

# Define the calculation function
def calculation(input1, input2, result_queue):
    print(f"Starting calculation for {input1} and {input2}")
    result = input1 + input2  # Replace this with your actual calculation
    print(f"Calculation result for {input1} and {input2} is {result}")
    result_queue.put(result)

# Define the inputs for the calculations
inputs = [(1, 2), (3, 4)]

# Create a Queue object
result_queue = queue.Queue()

while True:
    # Create threads for each calculation
    threads = [threading.Thread(target=calculation, args=(*input, result_queue)) for input in inputs]

    # Start the threads
    for thread in threads:
        thread.start()

    # Wait for all threads to complete
    for thread in threads:
        thread.join()

    # Get results from the queue
    while not result_queue.empty():
        print(f"Received result: {result_queue.get()}")

    # Pause before the next loop
    time.sleep(1)
