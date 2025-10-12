from controller import Robot, Lidar
import math
import heapq

# ====== PARAMETERS ======
TIME_STEP = 64
MAP_SIZE = 20          # Grid size (20x20 cells)
CELL_SIZE = 0.2        # Each cell = 0.2m → total map: 4m x 4m
OBSTACLE_THRESHOLD = 1.0  # LiDAR range below this = obstacle
START = (10, 10)       # Robot starts at center
GOAL = (17, 17)        # Goal position in grid coordinates

# ====== INIT ROBOT & LIIDAR ======
robot = Robot()
lidar = robot.getDevice("Hokuyo URG-04LX-UG01")
if lidar:
    lidar.enable(TIME_STEP)
    print("✓ LiDAR enabled")
else:
    print("✗ LiDAR NOT FOUND - Using dummy map!")
    lidar = None

# ====== MAP CLASS ======
class OccupancyGrid:
    def __init__(self, size, cell_size):
        self.size = size
        self.cell_size = cell_size
        self.grid = [[0 for _ in range(size)] for _ in range(size)]  # 0=free, 1=obstacle
        self.robot_pos = START

    def world_to_grid(self, x, y):
        """Convert real-world coordinates to grid indices"""
        row = int((y + self.size * self.cell_size / 2) / self.cell_size)
        col = int((x + self.size * self.cell_size / 2) / self.cell_size)
        return max(0, min(self.size - 1, row)), max(0, min(self.size - 1, col))

    def update_from_lidar(self):
        """Update grid using LiDAR point cloud"""
        if not lidar:
            return  # Skip if no LiDAR (for testing)

        try:
            ranges = lidar.getRangeImage()
            num_points = len(ranges)
            if num_points == 0:
                return

            # LiDAR field: typically 270 degrees (-135° to +135°)
            angle_min = -2.356  # radians
            angle_max = 2.356
            angle_step = (angle_max - angle_min) / num_points

            # Assume robot at origin (0,0), facing +Y direction
            for i in range(num_points):
                dist = ranges[i]
                if dist > OBSTACLE_THRESHOLD or dist == float('inf'):
                    continue

                angle = angle_min + i * angle_step
                # Convert polar to Cartesian (world coords)
                obs_x = dist * math.cos(angle)
                obs_y = dist * math.sin(angle)

                # Convert to grid
                r, c = self.world_to_grid(obs_x, obs_y)
                self.grid[r][c] = 1  # Mark as obstacle

                # Also mark nearby cells (to account for sensor error/robot size)
                for dr in [-1, 0, 1]:
                    for dc in [-1, 0, 1]:
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < self.size and 0 <= nc < self.size:
                            self.grid[nr][nc] = 1

        except Exception as e:
            print(f"⚠ LiDAR error: {e}")

    def a_star(self, start, goal):
        """A* algorithm to find shortest path"""
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance

        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}

        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 4-directional

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                if not (0 <= neighbor[0] < self.size and 0 <= neighbor[1] < self.size):
                    continue
                if self.grid[neighbor[0]][neighbor[1]] == 1:
                    continue

                tentative_g = g_score[current] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []  # No path found

    def print_map(self, path=[]):
        """Print ASCII map to terminal"""
        print("\n" + "="*60)
        print("🗺  OCCUPANCY GRID MAP (Top-left = (0,0))")
        print("S = Start | G = Goal | # = Obstacle | . = Free | * = Path | R = Robot")
        print("="*60)

        for r in range(self.size):
            row_str = ""
            for c in range(self.size):
                cell = self.grid[r][c]
                pos = (r, c)
                if pos == START:
                    row_str += " S "
                elif pos == GOAL:
                    row_str += " G "
                elif pos == self.robot_pos:
                    row_str += " R "
                elif pos in path:
                    row_str += " * "
                elif cell == 1:
                    row_str += " # "
                else:
                    row_str += " . "
            print(row_str)
        print("="*60)


# ====== MAIN LOOP ======
grid_map = OccupancyGrid(MAP_SIZE, CELL_SIZE)
step_count = 0
path = []

print("🤖 TIAGo Mapping & Shortest Path Finder")
print(f"Map: {MAP_SIZE}x{MAP_SIZE} ({MAP_SIZE * CELL_SIZE}m x {MAP_SIZE * CELL_SIZE}m)")
print(f"Start: {START}, Goal: {GOAL}")
print("Waiting for LiDAR data...")

while robot.step(TIME_STEP) != -1:
    step_count += 1

    # Update map every 10 steps (LiDAR scan rate)
    if step_count % 10 == 0:
        grid_map.update_from_lidar()

    # Replan path every 30 steps or if no path
    if step_count % 30 == 0 or not path:
        print(f"\n🔄 Replanning path at step {step_count}...")
        path = grid_map.a_star(START, GOAL)
        if path:
            print(f"✅ Found path with {len(path)} waypoints.")
        else:
            print("❌ No path found to goal.")

    # Print map every 50 steps
    if step_count % 50 == 0:
        grid_map.print_map(path)

    # Optional: Exit after 200 steps for demo
    if step_count >= 200:
        print("\n🏁 Simulation complete. Check final map above.")
        break