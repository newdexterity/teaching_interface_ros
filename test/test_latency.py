import requests
import time
import numpy as np


def test_latency(ip_address, cycles):
    """
    Measures the average response latency from the teaching interface.
    Args:
        ip_address(str): Assigned IP of the teaching interface.
        cycles(int): Number of request cycles to evaluate.

    Returns:
        float: average latency in seconds
    """
    # Create data array
    latency_arr = []

    # Perform the specified number of requests and parse data
    for i in range(cycles):
        time_start = time.time()
        r = requests.get('http://' + ip_address)
        data = r.json()
        time_end = time.time()
        latency_arr.append(time_end - time_start)

    # Average latency
    mean_latency = np.mean(latency_arr)

    return mean_latency


if __name__ == '__main__':
    avg = test_latency(ip_address='192.168.1.3', cycles=20)
    print("Average latency of teaching interface: {} s /  {} Hz.".format(avg, 1.0 / avg))
