import pygame
import numpy as np
from dataclasses import dataclass
import math
from collections import deque

import argparse, sys

@dataclass
class Boid:
    x: float
    y: float
    dx: float
    dy: float
    # Add a history of positions and velocities
    position_history: deque  # Will store (x, y) tuples
    velocity_history: deque  # Will store (dx, dy) tuples

class BoidSimulation:
    def __init__(self, width=1024, height=768, perception_delay=5, num_boids=100, vis_range=75):
        pygame.init()
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Boids with Perception Delay")
        
        self.boids = []
        self.num_boids = num_boids
        self.visual_range = vis_range
        self.visual_range_sq = self.visual_range * self.visual_range  # squared visual range for comparisons
        self.cell_size = self.visual_range  # each cell is roughly the size of the visual range
        self.clock = pygame.time.Clock()
        
        # Perception delay (number of frames)
        self.perception_delay = perception_delay

        # Add a flag to toggle trajectory display
        self.show_trajectories = False

        maxlen = perception_delay + 1
        
        # Initialize boids with random positions and velocities
        for _ in range(self.num_boids):
            # Initialize position and velocity history deques with initial values
            position_history = deque(maxlen=100)  # Limit history to 100 steps
            velocity_history = deque(maxlen=maxlen)  # Limit history to perception delay steps
            
            x = np.random.rand() * width
            y = np.random.rand() * height
            dx = np.random.rand() * 10 - 5
            dy = np.random.rand() * 10 - 5
            
            # Fill history with initial position and velocity
            for _ in range(self.perception_delay + 1):
                position_history.append((x, y))
                velocity_history.append((dx, dy))
            
            self.boids.append(Boid(
                x=x,
                y=y,
                dx=dx,
                dy=dy,
                position_history=position_history,
                velocity_history=velocity_history
            ))
    
    def get_neighbors_with_delay(self, boid, grid):
        """Return a list of neighboring boids (within visual range) using delayed position data."""
        neighbors = []
        current_cell_x = int(boid.x / self.cell_size)
        current_cell_y = int(boid.y / self.cell_size)
        
        # Check the boid's cell and all adjacent cells
        for i in range(current_cell_x - 1, current_cell_x + 2):
            for j in range(current_cell_y - 1, current_cell_y + 2):
                if (i, j) in grid:
                    for other in grid[(i, j)]:
                        if other is not boid:
                            # Use delayed position information
                            if len(other.position_history) > self.perception_delay:
                                delayed_x, delayed_y = other.position_history[-self.perception_delay]
                                
                                # Calculate distance using the delayed position
                                dx = boid.x - delayed_x
                                dy = boid.y - delayed_y
                                
                                if dx * dx + dy * dy < self.visual_range_sq:
                                    neighbors.append((other, delayed_x, delayed_y, 
                                                     other.velocity_history[-self.perception_delay]))
        
        return neighbors

    def keep_within_bounds(self, boid):
        margin = 200
        turn_factor = 1

        if boid.x < margin:
            boid.dx += turn_factor
        if boid.x > self.width - margin:
            boid.dx -= turn_factor
        if boid.y < margin:
            boid.dy += turn_factor
        if boid.y > self.height - margin:
            boid.dy -= turn_factor

    def fly_towards_center(self, boid, neighbors):
        centerX = 0
        centerY = 0
        num_neighbors = 0
        centering_factor = 0.005

        for other, delayed_x, delayed_y, _ in neighbors:
            centerX += delayed_x
            centerY += delayed_y
            num_neighbors += 1

        if num_neighbors:
            centerX /= num_neighbors
            centerY /= num_neighbors

            boid.dx += (centerX - boid.x) * centering_factor
            boid.dy += (centerY - boid.y) * centering_factor

    def avoid_others(self, boid, neighbors):
        min_distance = 20  
        min_distance_sq = min_distance * min_distance
        avoid_factor = 0.05 
        moveX = 0
        moveY = 0

        for other, delayed_x, delayed_y, _ in neighbors:
            dx = boid.x - delayed_x
            dy = boid.y - delayed_y
            if dx * dx + dy * dy < min_distance_sq:
                moveX += dx
                moveY += dy

        boid.dx += moveX * avoid_factor
        boid.dy += moveY * avoid_factor

    def match_velocity(self, boid, neighbors):
        matching_factor = 0.05
        avgDX = 0
        avgDY = 0
        num_neighbors = 0

        for other, _, _, delayed_velocity in neighbors:
            delayed_dx, delayed_dy = delayed_velocity
            avgDX += delayed_dx
            avgDY += delayed_dy
            num_neighbors += 1

        if num_neighbors:
            avgDX /= num_neighbors
            avgDY /= num_neighbors

            boid.dx += (avgDX - boid.dx) * matching_factor
            boid.dy += (avgDY - boid.dy) * matching_factor

    def limit_speed(self, boid):
        speed_limit = 10
        speed = math.sqrt(boid.dx * boid.dx + boid.dy * boid.dy)
        if speed > speed_limit:
            boid.dx = (boid.dx / speed) * speed_limit
            boid.dy = (boid.dy / speed) * speed_limit

    # Ensure the circle's center is within screen bounds, considering its radius
    def clamp_circle(self, x, y, radius):
        x = max(radius, min(self.width - radius, x))
        y = max(radius, min(self.height - radius, y))
        return x, y

    def update_boids(self):
        # Build a spatial grid (a dictionary mapping (cell_x, cell_y) -> list of boids)
        grid = {}
        for boid in self.boids:
            cell = (int(boid.x / self.cell_size), int(boid.y / self.cell_size))
            if cell not in grid:
                grid[cell] = []
            grid[cell].append(boid)
        
        # Update each boid using only nearby boids (neighbors) with delay
        for boid in self.boids:
            neighbors = self.get_neighbors_with_delay(boid, grid)
            self.fly_towards_center(boid, neighbors)
            self.avoid_others(boid, neighbors)
            self.match_velocity(boid, neighbors)
            self.keep_within_bounds(boid)
            self.limit_speed(boid)

            # Update position
            boid.x += boid.dx
            boid.y += boid.dy
            
            # Update position and velocity history
            boid.position_history.append((boid.x, boid.y))
            boid.velocity_history.append((boid.dx, boid.dy))

    def draw(self):
        self.screen.fill((0, 0, 0))
        
        for boid in self.boids:
            angle = math.atan2(boid.dy, boid.dx)
            size = 4
            
            # Draw the boid as a triangle
            points = [
                (boid.x + math.cos(angle) * size * 2,
                 boid.y + math.sin(angle) * size * 2),
                (boid.x + math.cos(angle + 2.4) * size,
                 boid.y + math.sin(angle + 2.4) * size),
                (boid.x + math.cos(angle - 2.4) * size,
                 boid.y + math.sin(angle - 2.4) * size)
            ]
            pygame.draw.polygon(self.screen, (85, 140, 244), points)
            
            # Optionally visualize the delayed position that other boids perceive
            if len(boid.position_history) >= self.perception_delay:
                delayed_x, delayed_y = boid.position_history[-self.perception_delay]
                delayed_x, delayed_y = self.clamp_circle(delayed_x, delayed_y, 2)
                pygame.draw.circle(self.screen, (255, 100, 100), (int(delayed_x), int(delayed_y)), 2)
            
            # Draw the trajectory if the flag is set
            if self.show_trajectories and len(boid.position_history) > 1:
                points = list(boid.position_history)
                for i in range(1, len(points)):
                    start_pos = points[i - 1]
                    end_pos = points[i]
                    # Calculate the fade color
                    fade_factor = i / len(points)
                    color = (int(100 * fade_factor), int(255 * fade_factor), int(100 * fade_factor))
                    pygame.draw.line(self.screen, color, start_pos, end_pos, 1)

        # Display the current perception delay on screen
        font = pygame.font.SysFont(None, 36)
        delay_text = font.render(f"Perception Delay: {self.perception_delay} frames", True, (255, 255, 255))
        self.screen.blit(delay_text, (10, 10))

        pygame.display.flip()

    def run(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    # Allow adjusting the perception delay with up/down arrow keys
                    elif event.key == pygame.K_UP:
                        self.perception_delay = min(10, self.perception_delay + 1)
                    elif event.key == pygame.K_DOWN:
                        self.perception_delay = max(1, self.perception_delay - 1)
                    # Toggle trajectory display with 't' key
                    elif event.key == pygame.K_t:
                        self.show_trajectories = not self.show_trajectories

            self.update_boids()
            self.draw()
            self.clock.tick(60)

        pygame.quit()

def main():
    # process command line arguments
    cmd = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    cmd.add_argument('--maxlen', type=int, default=35, help='max buffer length (obsolete), maxlen = delay+1');
    cmd.add_argument('--delay', type=int, default=1, help='delay');
    cmd.add_argument('--width', type=int, default=1024, help='arena width');
    cmd.add_argument('--height', type=int, default=768, help='arena height');
    cmd.add_argument('--num_boids', type=int, default=100, help='number of boids');
    cmd.add_argument('--vis_range', type=int, default=75, help='vis_range');
    args = cmd.parse_args()

    # set perception delay
    sim = BoidSimulation(perception_delay=args.delay, width=args.width, height=args.height, num_boids=args.num_boids, vis_range=args.vis_range)
    sim.run()

if __name__ == "__main__":
    main()
