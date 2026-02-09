import kachaka_api

# Connect to robot
robot_ip = "192.168.0.157"  # <--- REPLACE WITH YOUR IP
client = kachaka_api.KachakaApiClient(f"{robot_ip}:26400")

# Get all saved locations
locations = client.get_locations()

print(f"{'NAME':<15} | {'X (meters)':<10} | {'Y (meters)':<10}")
print("-" * 40)

for loc in locations:
    # location objects usually have a 'pose' attribute
    # accessing pose.x and pose.y
    try:
        name = loc.name
        x = round(loc.pose.x, 3)
        y = round(loc.pose.y, 3)
        print(f"{name:<15} | {x:<10} | {y:<10}")
    except AttributeError:
        pass