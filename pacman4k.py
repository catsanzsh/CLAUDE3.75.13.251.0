import pygame
import random
import asyncio
import platform
import math # Added for math.pi and math.radians
import sys  # Added for sys.modules check
import colorsys # For HSV to RGB conversion in vibe mode

# Initialize Pygame
pygame.init()
pygame.mixer.init()  # Initialize mixer for sound effects

# Constants
WIDTH = 30
HEIGHT = 20
CELL_SIZE = 20
SCREEN_WIDTH = WIDTH * CELL_SIZE
SCREEN_HEIGHT = HEIGHT * CELL_SIZE
FPS = 60

# Colors
WALL_COLOR = (0, 0, 255)
PELLET_COLOR = (255, 255, 255)
PACMAN_COLOR = (255, 255, 0)
GHOST_COLORS = [(255, 0, 0), (255, 184, 255), (0, 255, 255), (255, 184, 82)] # Red, Pink, Cyan, Orange
VULNERABLE_COLOR = (0, 0, 200) # Darker blue for vulnerable ghosts
EATEN_GHOST_COLOR = (100, 100, 100) # Color for eaten ghost returning to base

# Maze (30x20 grid)
# '.' denotes a path with a regular pellet
# 'o' denotes a path with a power pellet
# '#' denotes a wall
maze = [
    list("##############################"),  # 30 chars
    list("#...........#......#.........#"),  # 30 chars
    list("#.#########.#.####.#.#######.#"),  # 30 chars
    list("#.#.......#.#.#..#.#.#.....#.#"),  # 30 chars
    list("#.#.#####.#.#.#..#.#.#.###.#.#"),  # 30 chars
    list("#.#.#...#.#.#.#..#.#.#.#...#.#"),  # 30 chars
    list("#.#.#.###.#.#.####.#.#.###.#.#"),  # 30 chars
    list("#.#.#.....#.#......#.#.....#.#"),  # 30 chars
    list("#.#.######################.#.#"),  # 30 chars
    list("#.#........................#.#"),  # 30 chars - ghost start/respawn row
    list("#.#.######################.#.#"),  # 30 chars
    list("#.#.....#.#......#.#.....#.#.#"),  # 30 chars - fixed
    list("#.#.###.#.#.####.#.#.###.#.#.#"),  # 30 chars - fixed
    list("#.#.#...#.#.#..#.#.#.#...#.#.#"),  # 30 chars - fixed
    list("#.#.#####.#.#.#..#.#.#.###.#.#"),  # 30 chars
    list("#.#.......#.#.#..#.#.#.....#.#"),  # 30 chars
    list("#.#########.#.####.#.#######.#"),  # 30 chars
    list("#...........#......#.........#"),  # 30 chars
    list("##############################"),  # 30 chars
    list("##############################")   # 30 chars - Extra bottom border
]

# Add power pellets at specific locations
maze[3][5] = 'o'
maze[3][WIDTH - 6] = 'o'
maze[HEIGHT - 4][5] = 'o'
maze[HEIGHT - 4][WIDTH - 6] = 'o'

# Game state variables - these will be properly initialized in reset_game()
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Pac-Man Vibe Mode")
clock = pygame.time.Clock()

# These will be initialized by reset_game()
pacman_pos = [0.0, 0.0]
current_dir = [0, 0]
requested_dir = [0, 0]
ghosts = []
pellets = [] # 2D array to track pellet status
score = 0
lives = 3
power_timer = 0.0 # Float for more precise timing
game_state = "start" # Possible states: "start", "playing", "won", "lost"
running = True # Main game loop flag

font = pygame.font.Font(None, 36) # Default font for score, messages
small_font = pygame.font.Font(None, 24) # Smaller font for prompts or details
speed = 100.0  # pixels per second for Pac-Man

# Ghost spawn parameters
GHOST_SPAWN_Y = 9.5 * CELL_SIZE # Y-coordinate for ghost spawn/respawn (center of row 9)
GHOST_SPAWN_X_COORDS = [ # X-coordinates for initial ghost positions
    (WIDTH/2 - 2.5) * CELL_SIZE, # Blinky (Red)
    (WIDTH/2 - 1.5) * CELL_SIZE, # Pinky (Pink)
    (WIDTH/2 - 0.5) * CELL_SIZE, # Inky (Cyan) - also central respawn point
    (WIDTH/2 + 0.5) * CELL_SIZE, # Clyde (Orange)
]
GHOST_RESPAWN_POS = [GHOST_SPAWN_X_COORDS[2], GHOST_SPAWN_Y] # Central point for ghosts to return when eaten

POWER_DURATION = 7.0 # seconds for power pellet effect

# Vibe Mode specific variables
vibe_mode = False
vibe_color_offset = 0.0
vibe_pulse_value = 0.0
vibe_pulse_direction = 1
particles = []

# Sound effects
try:
    # Try to load sounds - they may not exist, so we'll handle the exception
    chomp_sound = pygame.mixer.Sound("chomp.wav")
    power_sound = pygame.mixer.Sound("power.wav")
    ghost_eat_sound = pygame.mixer.Sound("ghost_eat.wav")
    death_sound = pygame.mixer.Sound("death.wav")
    victory_sound = pygame.mixer.Sound("victory.wav")
    vibe_music = pygame.mixer.Sound("vibe_music.wav")
    sounds_loaded = True
except:
    sounds_loaded = False
    print("Sound files not found. Continuing without sound.")

# Helper Functions
def get_grid_pos(pixel_pos):
    """Convert pixel position (center of an entity) to grid coordinates (col, row)."""
    return int(pixel_pos[0] / CELL_SIZE), int(pixel_pos[1] / CELL_SIZE)

def hsv_to_rgb(h, s, v):
    """Convert HSV color values to RGB."""
    r, g, b = colorsys.hsv_to_rgb(h, s, v)
    return int(r * 255), int(g * 255), int(b * 255)

def get_vibe_color(base_color, offset=0.0, intensity=1.0):
    """Get a color modified by vibe mode."""
    if not vibe_mode:
        return base_color
    
    # Convert RGB to HSV, modify hue, convert back to RGB
    r, g, b = base_color
    h, s, v = colorsys.rgb_to_hsv(r/255, g/255, b/255)
    h = (h + offset) % 1.0
    s = min(1.0, s * intensity)
    v = min(1.0, v * intensity)
    
    return hsv_to_rgb(h, s, v)

def add_particle(x, y, color, size=3, lifetime=0.5, speed=30):
    """Add a particle effect to the game."""
    angle = random.uniform(0, 2 * math.pi)
    dx = math.cos(angle) * speed
    dy = math.sin(angle) * speed
    particles.append({
        'x': x, 'y': y,
        'dx': dx, 'dy': dy,
        'color': color,
        'size': size,
        'lifetime': lifetime,
        'max_lifetime': lifetime
    })

def update_particles(dt):
    """Update all particle effects."""
    global particles
    new_particles = []
    
    for p in particles:
        p['lifetime'] -= dt
        if p['lifetime'] <= 0:
            continue
            
        # Update position
        p['x'] += p['dx'] * dt
        p['y'] += p['dy'] * dt
        
        # Add to new list if still alive
        new_particles.append(p)
    
    particles = new_particles

def draw_particles():
    """Draw all particle effects."""
    for p in particles:
        # Fade out based on remaining lifetime
        alpha = int(255 * (p['lifetime'] / p['max_lifetime']))
        color = list(p['color'])
        if len(color) == 3:
            color.append(alpha)
        else:
            color[3] = alpha
            
        # Draw particle
        size = int(p['size'] * (0.5 + 0.5 * p['lifetime'] / p['max_lifetime']))
        pygame.draw.circle(screen, color, (int(p['x']), int(p['y'])), size)

def draw_maze():
    """Draw the maze walls and any remaining pellets."""
    global pellets, vibe_color_offset, vibe_pulse_value # Ensure access to the global variables
    
    # Calculate vibrant colors if vibe mode is on
    wall_color = get_vibe_color(WALL_COLOR, vibe_color_offset, 1.0 + vibe_pulse_value * 0.3)
    pellet_color = get_vibe_color(PELLET_COLOR, vibe_color_offset + 0.33, 1.0 + vibe_pulse_value * 0.3)
    power_pellet_color = get_vibe_color(PELLET_COLOR, vibe_color_offset + 0.67, 1.0 + vibe_pulse_value * 0.5)
    
    for y_idx in range(HEIGHT):
        for x_idx in range(WIDTH):
            rect = (x_idx * CELL_SIZE, y_idx * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            
            # Safety check for maze bounds
            is_wall = (y_idx < len(maze) and x_idx < len(maze[y_idx]) and maze[y_idx][x_idx] == '#')
            
            if is_wall:
                pygame.draw.rect(screen, wall_color, rect)
            elif y_idx < len(pellets) and x_idx < len(pellets[y_idx]) and pellets[y_idx][x_idx]: 
                # Check if pellet at (y_idx, x_idx) still exists
                center_x = x_idx * CELL_SIZE + CELL_SIZE // 2
                center_y = y_idx * CELL_SIZE + CELL_SIZE // 2
                
                # Determine if this is a power pellet
                is_power_pellet = (y_idx < len(maze) and x_idx < len(maze[y_idx]) and maze[y_idx][x_idx] == 'o')
                
                # Power pellets ('o') are larger than regular pellets ('.')
                pellet_radius = CELL_SIZE // 4 if is_power_pellet else CELL_SIZE // 8
                
                # Use power pellet color for power pellets
                current_pellet_color = power_pellet_color if is_power_pellet else pellet_color
                
                # Pellet pulses in vibe mode
                if vibe_mode and is_power_pellet:
                    pellet_radius += int(vibe_pulse_value * CELL_SIZE // 6)
                
                pygame.draw.circle(screen, current_pellet_color, (center_x, center_y), pellet_radius)

def draw_pacman():
    """Draw Pac-Man with an animated mouth."""
    global pacman_pos, current_dir, vibe_color_offset, vibe_pulse_value # Access global Pac-Man state
    center_x, center_y = int(pacman_pos[0]), int(pacman_pos[1])
    radius = CELL_SIZE // 2 - 1 # Slightly smaller than cell to avoid visual overlap with walls

    # Get Pac-Man color, possibly modified by vibe mode
    pacman_color = get_vibe_color(PACMAN_COLOR, vibe_color_offset, 1.0 + vibe_pulse_value * 0.3)

    # Mouth animation: angle changes based on time if Pac-Man is moving
    mouth_angle_degrees = 0
    if current_dir != [0,0]: # Only animate mouth if moving
        mouth_angle_degrees = (pygame.time.get_ticks() // 150 % 2) * 25 # Oscillates between 0 and 25 degrees

    # Draw Pac-Man's body (yellow circle)
    pygame.draw.circle(screen, pacman_color, (center_x, center_y), radius)

    # Draw mouth if it's supposed to be open
    if mouth_angle_degrees > 0:
        # Determine base angle based on current direction
        # Pygame angles: 0 is right, 90 is down, 180 is left, 270 is up
        if current_dir[0] > 0: angle_degrees = 0    # Facing Right
        elif current_dir[0] < 0: angle_degrees = 180 # Facing Left
        elif current_dir[1] > 0: angle_degrees = 270 # Facing Down
        elif current_dir[1] < 0: angle_degrees = 90  # Facing Up
        else: angle_degrees = 0 # Default: if not moving, mouth (if open) faces right

        base_angle_rad = math.radians(angle_degrees)
        mouth_half_angle_rad = math.radians(mouth_angle_degrees / 2)

        # Calculate points for the mouth polygon (a triangle)
        p1 = (center_x, center_y) # Center of the circle (apex of the triangle)
        # Point on the circle edge for one side of the mouth
        p2 = (center_x + radius * math.cos(base_angle_rad - mouth_half_angle_rad),
              center_y + radius * math.sin(base_angle_rad - mouth_half_angle_rad))
        # Point on the circle edge for the other side of the mouth
        p3 = (center_x + radius * math.cos(base_angle_rad + mouth_half_angle_rad),
              center_y + radius * math.sin(base_angle_rad + mouth_half_angle_rad))
        
        pygame.draw.polygon(screen, (0,0,0), [p1, p2, p3]) # Draw black triangle for mouth


def draw_ghost(ghost_data):
    """Draw a single ghost, changing color if vulnerable or eaten."""
    global power_timer, pacman_pos, vibe_color_offset, vibe_pulse_value # Access global game state
    center_x, center_y = int(ghost_data['pos'][0]), int(ghost_data['pos'][1])
    radius = CELL_SIZE // 2 - 2 # Ghost radius, slightly smaller than Pac-Man

    current_display_color = ghost_data['color'] # Default color
    if ghost_data["is_eaten"]:
        current_display_color = EATEN_GHOST_COLOR # Gray when returning to base
    elif power_timer > 0: # If power pellet is active
        # Flash white if power is about to run out (e.g., last 3 seconds)
        if power_timer < 3 and (pygame.time.get_ticks() // 250 % 2 == 0):
            current_display_color = PELLET_COLOR # Flash white
        else:
            current_display_color = VULNERABLE_COLOR # Dark blue when vulnerable
    
    # Apply vibe mode color transformation if active
    if vibe_mode and not ghost_data["is_eaten"] and power_timer <= 0:
        # Only apply to active ghosts in normal state
        current_display_color = get_vibe_color(current_display_color, 
                                               vibe_color_offset + ghost_data['id'] * 0.25, 
                                               1.0 + vibe_pulse_value * 0.3)

    # Draw ghost body (circle)
    pygame.draw.circle(screen, current_display_color, (center_x, center_y), radius)
    # Draw ghost "skirt" (rectangle below the circle)
    skirt_rect = pygame.Rect(center_x - radius, center_y, radius * 2, radius)
    pygame.draw.rect(screen, current_display_color, skirt_rect)
    # Add wavy bottom to the skirt using small circles
    for i in range(3): # Three bumps for the wavy effect
        bump_center_x = center_x - radius + (radius * 2 / 3 * i) + (radius / 3)
        pygame.draw.circle(screen, current_display_color, (int(bump_center_x), center_y + radius), int(radius / 3) + 1)

    # Eyes (only draw if not eaten, or draw simplified eyes if eaten)
    eye_radius_val = radius // 3
    pupil_radius_val = eye_radius_val // 2
    eye_offset_x_val = radius // 2.5 # Horizontal offset for eyes from center

    # Calculate eye positions
    left_eye_center_x = int(center_x - eye_offset_x_val)
    left_eye_center_y = int(center_y - radius // 3)
    right_eye_center_x = int(center_x + eye_offset_x_val)
    right_eye_center_y = int(center_y - radius // 3)

    # Draw white part of eyes
    pygame.draw.circle(screen, (255, 255, 255), (left_eye_center_x, left_eye_center_y), eye_radius_val)
    pygame.draw.circle(screen, (255, 255, 255), (right_eye_center_x, right_eye_center_y), eye_radius_val)

    # Pupils (look towards Pac-Man if not eaten, otherwise fixed or not drawn)
    pupil_offset_x, pupil_offset_y = 0,0
    if not ghost_data["is_eaten"]: # Pupils follow Pac-Man if ghost is active
        dir_to_pac_x = pacman_pos[0] - ghost_data['pos'][0]
        dir_to_pac_y = pacman_pos[1] - ghost_data['pos'][1]
        dist_squared = dir_to_pac_x**2 + dir_to_pac_y**2
        if dist_squared > 0: # Avoid division by zero if Pac-Man is on ghost
            dist = math.sqrt(dist_squared)
            # Calculate pupil offset to "look" towards Pac-Man
            pupil_offset_x = (dir_to_pac_x / dist) * (eye_radius_val - pupil_radius_val) * 0.5
            pupil_offset_y = (dir_to_pac_y / dist) * (eye_radius_val - pupil_radius_val) * 0.5
    
    # Draw black pupils
    pygame.draw.circle(screen, (0,0,0), (int(left_eye_center_x + pupil_offset_x), int(left_eye_center_y + pupil_offset_y)), pupil_radius_val)
    pygame.draw.circle(screen, (0,0,0), (int(right_eye_center_x + pupil_offset_x), int(right_eye_center_y + pupil_offset_y)), pupil_radius_val)


def update_pacman(dt):
    """Move Pac-Man, handle turning, wall collisions, and pellet collection."""
    global current_dir, requested_dir, score, power_timer, pellets, pacman_pos, ghosts, vibe_mode # Ensure globals are accessible

    pac_current_grid_x, pac_current_grid_y = get_grid_pos(pacman_pos)

    # Try to change direction if a new direction is requested and Pac-Man is aligned
    if requested_dir != [0,0] and requested_dir != current_dir:
        # Tolerance for alignment: Pac-Man must be close to the center of a grid path to turn
        alignment_tolerance = speed * dt * 0.7 # Allow turning if slightly off-center (adjust as needed)

        if requested_dir[0] != 0:  # Horizontal turn requested
            # Must be vertically aligned in a corridor
            if abs(pacman_pos[1] - (pac_current_grid_y + 0.5) * CELL_SIZE) < alignment_tolerance:
                # Check if the path in the new direction is clear
                next_check_grid_x = pac_current_grid_x + requested_dir[0]
                # Safety check for maze bounds
                if (0 <= next_check_grid_x < WIDTH and 
                    0 <= pac_current_grid_y < len(maze) and 
                    next_check_grid_x < len(maze[pac_current_grid_y]) and 
                    maze[pac_current_grid_y][next_check_grid_x] != '#'):
                    current_dir = list(requested_dir) # Commit to new direction
                    pacman_pos[1] = (pac_current_grid_y + 0.5) * CELL_SIZE # Snap to vertical center of corridor
        elif requested_dir[1] != 0:  # Vertical turn requested
            # Must be horizontally aligned in a corridor
            if abs(pacman_pos[0] - (pac_current_grid_x + 0.5) * CELL_SIZE) < alignment_tolerance:
                # Check if the path in the new direction is clear
                next_check_grid_y = pac_current_grid_y + requested_dir[1]
                # Safety check for maze bounds
                if (0 <= next_check_grid_y < HEIGHT and
                    0 <= pac_current_grid_x < len(maze[0]) and
                    next_check_grid_y < len(maze) and 
                    maze[next_check_grid_y][pac_current_grid_x] != '#'):
                    current_dir = list(requested_dir) # Commit to new direction
                    pacman_pos[0] = (pac_current_grid_x + 0.5) * CELL_SIZE # Snap to horizontal center of corridor
    
    # Calculate potential new position based on current direction and speed
    potential_next_x = pacman_pos[0] + current_dir[0] * speed * dt
    potential_next_y = pacman_pos[1] + current_dir[1] * speed * dt

    # Wall collision detection and response
    # Check based on the leading edge of Pac-Man
    if current_dir[0] != 0: # Moving horizontally
        # Determine the grid cell of the leading edge
        leading_edge_pixel_x = potential_next_x + current_dir[0] * (CELL_SIZE / 2 * 0.95) # Project slightly ahead
        collision_check_grid_x = int(leading_edge_pixel_x / CELL_SIZE)
        # Y-grid for collision check should be the one Pac-Man is currently centered in
        path_center_grid_y = int(round(pacman_pos[1] / CELL_SIZE)) # Use rounded grid Y for current path

        # Boundary and wall check with safety bounds checking
        if not (0 <= collision_check_grid_x < WIDTH) or \
           (0 <= path_center_grid_y < len(maze) and 
            0 <= collision_check_grid_x < len(maze[path_center_grid_y]) and 
            maze[path_center_grid_y][collision_check_grid_x] == '#'):
            # Collision detected: snap Pac-Man to the edge of the current cell and stop
            pacman_pos[0] = (pac_current_grid_x + 0.5) * CELL_SIZE
            current_dir = [0, 0] # Stop movement
        else:
            pacman_pos[0] = potential_next_x # No collision, update position

    if current_dir[1] != 0: # Moving vertically
        leading_edge_pixel_y = potential_next_y + current_dir[1] * (CELL_SIZE / 2 * 0.95)
        collision_check_grid_y = int(leading_edge_pixel_y / CELL_SIZE)
        path_center_grid_x = int(round(pacman_pos[0] / CELL_SIZE))

        # Boundary and wall check with safety bounds checking
        if not (0 <= collision_check_grid_y < HEIGHT) or \
           (0 <= path_center_grid_x < len(maze[0]) and 
            0 <= collision_check_grid_y < len(maze) and 
            maze[collision_check_grid_y][path_center_grid_x] == '#'):
            pacman_pos[1] = (pac_current_grid_y + 0.5) * CELL_SIZE
            current_dir = [0, 0]
        else:
            pacman_pos[1] = potential_next_y
            
    # Screen wrapping (tunnels on sides) - Pac-Man exits one side and appears on the other
    # This simple version wraps at screen edges; a real Pac-Man would have specific tunnel cells.
    if pacman_pos[0] < -CELL_SIZE / 2: # Wrapped past left edge
        pacman_pos[0] = SCREEN_WIDTH - CELL_SIZE / 2.1 # Appear on right edge
    elif pacman_pos[0] > SCREEN_WIDTH + CELL_SIZE / 2: # Wrapped past right edge
        pacman_pos[0] = -CELL_SIZE / 2.1 # Appear on left edge

    # Collect pellets
    # Get grid position of Pac-Man's center for pellet collection
    collect_grid_x, collect_grid_y = get_grid_pos(pacman_pos)
    if (0 <= collect_grid_x < WIDTH and 
        0 <= collect_grid_y < HEIGHT and 
        collect_grid_y < len(pellets) and 
        collect_grid_x < len(pellets[collect_grid_y])): # Ensure within maze and pellets bounds
        
        if pellets[collect_grid_y][collect_grid_x]: # If a pellet exists at this location
            # Get pellet type with safety check
            pellet_type = '.'  # Default to regular pellet
            if (collect_grid_y < len(maze) and 
                collect_grid_x < len(maze[collect_grid_y])):
                pellet_type = maze[collect_grid_y][collect_grid_x]
                
            pellets[collect_grid_y][collect_grid_x] = False # Mark pellet as collected

            # Play sound if available
            if sounds_loaded:
                if pellet_type == 'o': # Power pellet
                    pygame.mixer.Sound.play(power_sound)
                else: # Regular pellet
                    pygame.mixer.Sound.play(chomp_sound)

            # Add particle effects in vibe mode
            if vibe_mode:
                pellet_color = PELLET_COLOR
                num_particles = 10 if pellet_type == 'o' else 3
                for _ in range(num_particles):
                    add_particle(pacman_pos[0], pacman_pos[1], 
                                get_vibe_color(pellet_color, random.random(), 1.2),
                                size=random.randint(2, 5),
                                lifetime=random.uniform(0.3, 0.8),
                                speed=random.uniform(20, 40))

            if pellet_type == 'o': # Power pellet
                score += 50
                power_timer = POWER_DURATION # Activate power mode
                # Make all non-eaten ghosts vulnerable
                for g_data in ghosts:
                    if not g_data["is_eaten"]: # Only affect ghosts that aren't already returning to base
                         g_data["is_eaten"] = False # Ensure visual state is vulnerable
            else: # Regular pellet
                score += 10

def update_ghosts(dt):
    """Move ghosts, handle their AI, wall collisions, and Pac-Man collision."""
    global score, lives, game_state, power_timer, pacman_pos, ghosts, current_dir, requested_dir # Ensure globals

    for ghost_data in ghosts:
        # Determine ghost speed: slower by default, faster when eaten, slower when vulnerable
        ghost_speed_factor = 0.65 # Default speed relative to Pac-Man
        if ghost_data["is_eaten"]:
            ghost_speed_factor = 1.2 # Faster when returning to base
        elif power_timer > 0:
            ghost_speed_factor = 0.4 # Slower when vulnerable

        current_ghost_speed = speed * ghost_speed_factor
        ghost_current_grid_x, ghost_current_grid_y = get_grid_pos(ghost_data['pos'])

        # --- Ghost AI State: EATEN (returning to base) ---
        if ghost_data["is_eaten"]:
            target_pixel_x, target_pixel_y = GHOST_RESPAWN_POS[0], GHOST_RESPAWN_POS[1]
            # Check if ghost has reached the respawn point
            if abs(ghost_data['pos'][0] - target_pixel_x) < CELL_SIZE * 0.6 and \
               abs(ghost_data['pos'][1] - target_pixel_y) < CELL_SIZE * 0.6: # Tolerance for arrival
                ghost_data["is_eaten"] = False # Revert to normal state
                ghost_data['pos'] = list(GHOST_RESPAWN_POS) # Snap to exact respawn position
                ghost_data['dir'] = [0,0] # Stop, will pick new direction next update
            else: # Move towards respawn point
                delta_x = target_pixel_x - ghost_data['pos'][0]
                delta_y = target_pixel_y - ghost_data['pos'][1]
                dist_sq = delta_x**2 + delta_y**2
                if dist_sq > 0: # Avoid division by zero
                    dist = math.sqrt(dist_sq)
                    ghost_data['dir'] = [delta_x / dist, delta_y / dist] # Normalized direction vector
                else: # Should be caught by arrival check, but as fallback:
                    ghost_data['dir'] = [0,0]
        
        # --- Ghost AI State: ACTIVE (chasing or wandering) ---
        else:
            # Simplified AI: Change direction at intersections or randomly
            # Check if ghost is near the center of a grid cell to make a turning decision
            is_at_intersection = (abs(ghost_data['pos'][0] - (ghost_current_grid_x + 0.5) * CELL_SIZE) < current_ghost_speed * dt * 1.1 and \
                                 abs(ghost_data['pos'][1] - (ghost_current_grid_y + 0.5) * CELL_SIZE) < current_ghost_speed * dt * 1.1)

            if is_at_intersection and (random.random() < 0.1 or ghost_data['dir'] == [0,0]): # Chance to change dir or if stopped
                # Snap to grid center before choosing new direction for precision
                ghost_data['pos'][0] = (ghost_current_grid_x + 0.5) * CELL_SIZE
                ghost_data['pos'][1] = (ghost_current_grid_y + 0.5) * CELL_SIZE

                valid_new_dirs = []
                for d_x, d_y in [[-1,0], [1,0], [0,-1], [0,1]]: # Left, Right, Up, Down
                    # Avoid immediate reversal unless no other option
                    if [d_x, d_y] == [-ghost_data['dir'][0], -ghost_data['dir'][1]] and ghost_data['dir'] != [0,0]:
                        # Count how many other valid paths exist
                        other_options = 0
                        for od_x, od_y in [[-1,0], [1,0], [0,-1], [0,1]]:
                            if [od_x, od_y] == [-ghost_data['dir'][0], -ghost_data['dir'][1]]: continue
                            next_temp_g_grid_x = ghost_current_grid_x + od_x
                            next_temp_g_grid_y = ghost_current_grid_y + od_y
                            
                            # Add bounds checking
                            if (0 <= next_temp_g_grid_x < WIDTH and 
                                0 <= next_temp_g_grid_y < HEIGHT and 
                                next_temp_g_grid_y < len(maze) and
                                next_temp_g_grid_x < len(maze[next_temp_g_grid_y]) and
                                maze[next_temp_g_grid_y][next_temp_g_grid_x] != '#'):
                                other_options +=1
                        if other_options > 0: continue # Only allow reversal if it's a dead end

                    next_g_grid_x_check = ghost_current_grid_x + d_x
                    next_g_grid_y_check = ghost_current_grid_y + d_y
                    
                    # Add bounds checking
                    if (0 <= next_g_grid_x_check < WIDTH and 
                        0 <= next_g_grid_y_check < HEIGHT and
                        next_g_grid_y_check < len(maze) and
                        next_g_grid_x_check < len(maze[next_g_grid_y_check]) and
                        maze[next_g_grid_y_check][next_g_grid_x_check] != '#'):
                        valid_new_dirs.append([d_x,d_y])
                
                if valid_new_dirs:
                    ghost_data['dir'] = random.choice(valid_new_dirs)
                # If stuck (no valid new dirs and was trying to turn), allow reversal (handled by next pass if needed)


        # --- Ghost Movement and Wall Collision (applies to both AI states) ---
        potential_ghost_next_x = ghost_data['pos'][0] + ghost_data['dir'][0] * current_ghost_speed * dt
        potential_ghost_next_y = ghost_data['pos'][1] + ghost_data['dir'][1] * current_ghost_speed * dt

        if ghost_data['dir'][0] != 0: # Moving horizontally
            ghost_leading_edge_x = potential_ghost_next_x + ghost_data['dir'][0] * (CELL_SIZE / 2 * 0.9)
            ghost_collision_grid_x = int(ghost_leading_edge_x / CELL_SIZE)
            ghost_path_grid_y = int(round(ghost_data['pos'][1] / CELL_SIZE)) # Current vertical path

            # Add bounds checking
            if not (0 <= ghost_collision_grid_x < WIDTH) or \
               (0 <= ghost_path_grid_y < len(maze) and 
                0 <= ghost_collision_grid_x < len(maze[ghost_path_grid_y]) and 
                maze[ghost_path_grid_y][ghost_collision_grid_x] == '#'):
                ghost_data['pos'][0] = (ghost_current_grid_x + 0.5) * CELL_SIZE # Snap to current cell center
                ghost_data['dir'] = [0,0] # Stop, will pick new direction
            else:
                ghost_data['pos'][0] = potential_ghost_next_x
        
        if ghost_data['dir'][1] != 0: # Moving vertically
            ghost_leading_edge_y = potential_ghost_next_y + ghost_data['dir'][1] * (CELL_SIZE / 2 * 0.9)
            ghost_collision_grid_y = int(ghost_leading_edge_y / CELL_SIZE)
            ghost_path_grid_x = int(round(ghost_data['pos'][0] / CELL_SIZE)) # Current horizontal path

            # Add bounds checking
            if not (0 <= ghost_collision_grid_y < HEIGHT) or \
               (0 <= ghost_path_grid_x < len(maze[0]) and 
                0 <= ghost_collision_grid_y < len(maze) and 
                maze[ghost_collision_grid_y][ghost_path_grid_x] == '#'):
                ghost_data['pos'][1] = (ghost_current_grid_y + 0.5) * CELL_SIZE # Snap
                ghost_data['dir'] = [0,0]
            else:
                ghost_data['pos'][1] = potential_ghost_next_y

        # --- Collision with Pac-Man ---
        # Use a slightly smaller collision radius for fairness
        collision_threshold = CELL_SIZE * 0.7 
        if abs(ghost_data['pos'][0] - pacman_pos[0]) < collision_threshold and \
           abs(ghost_data['pos'][1] - pacman_pos[1]) < collision_threshold:
            
            if power_timer > 0 and not ghost_data["is_eaten"]: # Pac-Man eats ghost
                score += 200 
                ghost_data["is_eaten"] = True
                # Play ghost eat sound if available
                if sounds_loaded:
                    pygame.mixer.Sound.play(ghost_eat_sound)
                    
                # Add particle effect in vibe mode
                if vibe_mode:
                    for _ in range(15):
                        add_particle(ghost_data['pos'][0], ghost_data['pos'][1], 
                                     get_vibe_color(ghost_data['color'], random.random(), 1.2),
                                     size=random.randint(3, 7),
                                     lifetime=random.uniform(0.5, 1.2),
                                     speed=random.uniform(30, 60))
                # Ghost will start returning to base (handled by its AI state change)
            elif not ghost_data["is_eaten"]: # Ghost catches Pac-Man (and not vulnerable)
                lives -= 1
                power_timer = 0 # Pac-Man loses power-up
                
                # Play death sound if available
                if sounds_loaded:
                    pygame.mixer.Sound.play(death_sound)
                
                # Reset Pac-Man's position and direction
                pacman_pos[:] = [1.5 * CELL_SIZE, 1.5 * CELL_SIZE] # Back to start
                current_dir[:] = [0, 0]
                requested_dir[:] = [0,0]
                
                # Reset Ghosts to their initial positions (or a "scatter" mode could be added)
                for i, g_reset in enumerate(ghosts):
                    g_reset["pos"] = [GHOST_SPAWN_X_COORDS[i], GHOST_SPAWN_Y]
                    g_reset["dir"] = [0,0]
                    g_reset["is_eaten"] = False

                if lives <= 0:
                    game_state = "lost"
                    return False # Signal to update_loop that game is over
                else: # Lost a life, but game continues
                    # Brief pause or visual cue for losing a life
                    screen.fill((100,0,0)) # Temporary red flash
                    pygame.display.flip()
                    pygame.time.wait(700) # Pause for 0.7 seconds
                    # The game will then redraw in the main loop
                    return True # Game continues, but positions reset

    return True # Game continues (no game-ending collision this frame)

def update_vibe_mode(dt):
    """Update vibe mode visual effects."""
    global vibe_color_offset, vibe_pulse_value, vibe_pulse_direction
    
    if not vibe_mode:
        return
        
    # Update color cycling
    vibe_color_offset = (vibe_color_offset + dt * 0.1) % 1.0
    
    # Update pulsing effect
    vibe_pulse_value += dt * vibe_pulse_direction * 0.5
    if vibe_pulse_value > 1.0:
        vibe_pulse_value = 1.0
        vibe_pulse_direction = -1
    elif vibe_pulse_value < 0.0:
        vibe_pulse_value = 0.0
        vibe_pulse_direction = 1

def update_loop():
    """Main game loop: handles events, updates game state, and draws everything."""
    global power_timer, running, game_state, current_dir, requested_dir, score, lives, pellets
    global ghosts, pacman_pos, vibe_mode # Globals

    dt = clock.tick(FPS) / 1000.0 # Delta time in seconds for frame-rate independent movement

    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            return # Exit update_loop immediately
        elif event.type == pygame.KEYDOWN:
            if game_state == "start": # On start screen
                if event.key == pygame.K_RETURN or event.key == pygame.K_SPACE:
                    reset_game() # Reset before starting
                    game_state = "playing"
            elif game_state == "playing": # During gameplay
                if event.key == pygame.K_LEFT: requested_dir = [-1, 0]
                elif event.key == pygame.K_RIGHT: requested_dir = [1, 0]
                elif event.key == pygame.K_UP: requested_dir = [0, -1]
                elif event.key == pygame.K_DOWN: requested_dir = [0, 1]
                # Toggle vibe mode with V key
                elif event.key == pygame.K_v:
                    vibe_mode = not vibe_mode
                    # Play/stop vibe music if available
                    if sounds_loaded:
                        try:
                            if vibe_mode:
                                pygame.mixer.Sound.play(vibe_music, loops=-1)  # Loop indefinitely
                            else:
                                pygame.mixer.Sound.stop(vibe_music)
                        except Exception as e:
                            print(f"Sound error: {e}")
            elif game_state in ["won", "lost"]: # On win/loss screen
                if event.key == pygame.K_RETURN or event.key == pygame.K_SPACE:
                    reset_game() # Reset for a new game
                    game_state = "playing" # Go directly to playing

    # --- Game State Logic ---
    if game_state == "start":
        screen.fill((0, 0, 0)) # Black background
        title_text_surf = font.render("Pac-Man Vibe Mode", True, PACMAN_COLOR)
        prompt_text_surf = small_font.render("Press Enter/Space to Start", True, PELLET_COLOR)
        vibe_text_surf = small_font.render("Press V during game to toggle Vibe Mode!", True, 
                                          (200, 100, 255) if (pygame.time.get_ticks() // 500) % 2 else (100, 200, 255))
        
        screen.blit(title_text_surf, (SCREEN_WIDTH // 2 - title_text_surf.get_width() // 2, SCREEN_HEIGHT // 2 - 50))
        screen.blit(prompt_text_surf, (SCREEN_WIDTH // 2 - prompt_text_surf.get_width() // 2, SCREEN_HEIGHT // 2 + 10))
        screen.blit(vibe_text_surf, (SCREEN_WIDTH // 2 - vibe_text_surf.get_width() // 2, SCREEN_HEIGHT // 2 + 40))
    
    elif game_state == "playing":
        # Update vibe mode effects
        update_vibe_mode(dt)
        if vibe_mode:
            update_particles(dt)
        
        update_pacman(dt)
        
        # Check for win condition: all pellets eaten
        pellets_remaining = False
        for row in pellets:
            if any(row):
                pellets_remaining = True
                break
        
        if not pellets_remaining:  # No pellets remain
            game_state = "won"
            # Play victory sound if available
            if sounds_loaded:
                try:
                    pygame.mixer.Sound.play(victory_sound)
                    pygame.mixer.Sound.stop(vibe_music)  # Stop vibe music if playing
                except Exception as e:
                    print(f"Sound error: {e}")
            # No immediate return; win screen will be drawn below

        # Update ghosts and check for game over from ghost collision
        if not update_ghosts(dt): # update_ghosts returns False if Pac-Man loses all lives
            # game_state is already set to "lost" by update_ghosts
            # Stop vibe music if playing
            if sounds_loaded:
                try:
                    pygame.mixer.Sound.stop(vibe_music)
                except Exception as e:
                    print(f"Sound error: {e}")

        # Update power pellet timer
        if power_timer > 0:
            power_timer -= dt
            if power_timer < 0:
                power_timer = 0 # Ensure it doesn't go negative

        # --- Drawing ---
        # In vibe mode, use a pulsing background instead of pure black
        if vibe_mode:
            # Calculate background color based on vibe pulse
            bg_intensity = int(20 + vibe_pulse_value * 15)
            screen.fill((bg_intensity, bg_intensity // 2, bg_intensity))
        else:
            screen.fill((0, 0, 0))  # Standard black background
            
        draw_maze()
        
        # Draw particles if in vibe mode
        if vibe_mode:
            draw_particles()
            
        draw_pacman()
        for ghost_data in ghosts:
            draw_ghost(ghost_data)
        
        # UI: Score and Lives display
        score_surf = font.render(f"Score: {score}", True, PELLET_COLOR)
        lives_surf = font.render(f"Lives: {lives}", True, PELLET_COLOR)
        screen.blit(score_surf, (10, 10))
        screen.blit(lives_surf, (SCREEN_WIDTH - lives_surf.get_width() - 10, 10))

        # UI: Power timer display
        if power_timer > 0:
            power_text_surf = small_font.render(f"Power: {int(math.ceil(power_timer))}s", True, VULNERABLE_COLOR)
            screen.blit(power_text_surf, (SCREEN_WIDTH // 2 - power_text_surf.get_width() // 2, 10))
            
        # UI: Vibe mode indicator
        vibe_text = "VIBE MODE: ON" if vibe_mode else "VIBE MODE: OFF (Press V)"
        vibe_color = get_vibe_color((200, 100, 255), vibe_color_offset) if vibe_mode else (150, 150, 150)
        vibe_surf = small_font.render(vibe_text, True, vibe_color)
        screen.blit(vibe_surf, (SCREEN_WIDTH // 2 - vibe_surf.get_width() // 2, SCREEN_HEIGHT - 20))

    elif game_state == "won":
        # Create a vibrant win screen if vibe mode was on
        if vibe_mode:
            # Use pulsing rainbow background
            h = (pygame.time.get_ticks() % 3000) / 3000.0  # Cycle through hues
            rgb = hsv_to_rgb(h, 0.8, 0.3)  # Less saturated for background
            screen.fill(rgb)
            
            # Add particle celebration
            if random.random() < 0.2:  # Occasionally add new particles
                for _ in range(5):
                    particle_h = random.random()
                    particle_color = hsv_to_rgb(particle_h, 1.0, 1.0)
                    add_particle(
                        random.randint(0, SCREEN_WIDTH),
                        random.randint(0, SCREEN_HEIGHT),
                        particle_color,
                        size=random.randint(4, 8),
                        lifetime=random.uniform(0.8, 1.5),
                        speed=random.uniform(20, 60)
                    )
            
            # Update and draw particles
            update_particles(dt)
            draw_particles()
            
            # Vibrant text
            win_text_surf = font.render("You Win!", True, hsv_to_rgb((h + 0.5) % 1.0, 1.0, 1.0))
        else:
            screen.fill((0, 50, 0)) # Dark green background for win
            win_text_surf = font.render("You Win!", True, (0, 255, 0)) # Bright green text
            
        final_score_surf = small_font.render(f"Final Score: {score}", True, PELLET_COLOR)
        restart_prompt_surf = small_font.render("Press Enter/Space to Play Again", True, (200, 200, 200))
        screen.blit(win_text_surf, (SCREEN_WIDTH // 2 - win_text_surf.get_width() // 2, SCREEN_HEIGHT // 2 - 40))
        screen.blit(final_score_surf, (SCREEN_WIDTH // 2 - final_score_surf.get_width() // 2, SCREEN_HEIGHT // 2))
        screen.blit(restart_prompt_surf, (SCREEN_WIDTH // 2 - restart_prompt_surf.get_width() // 2, SCREEN_HEIGHT // 2 + 40))

    elif game_state == "lost":
        screen.fill((50, 0, 0)) # Dark red background for game over
        lose_text_surf = font.render("Game Over", True, (255, 0, 0)) # Bright red text
        final_score_surf = small_font.render(f"Final Score: {score}", True, PELLET_COLOR)
        restart_prompt_surf = small_font.render("Press Enter/Space to Play Again", True, (200, 200, 200))
        screen.blit(lose_text_surf, (SCREEN_WIDTH // 2 - lose_text_surf.get_width() // 2, SCREEN_HEIGHT // 2 - 40))
        screen.blit(final_score_surf, (SCREEN_WIDTH // 2 - final_score_surf.get_width() // 2, SCREEN_HEIGHT // 2))
        screen.blit(restart_prompt_surf, (SCREEN_WIDTH // 2 - restart_prompt_surf.get_width() // 2, SCREEN_HEIGHT // 2 + 40))

    pygame.display.flip() # Update the full display

def reset_game():
    """Resets all game variables to their initial states for a new game."""
    global pacman_pos, current_dir, requested_dir, ghosts, pellets, score, lives, power_timer, game_state, running
    global vibe_mode, particles  # Include vibe mode variables

    pacman_pos = [1.5 * CELL_SIZE, 1.5 * CELL_SIZE] # Start Pac-Man in cell (1,1)
    current_dir = [0, 0] # Initially not moving
    requested_dir = [0, 0] 
    
    # Initialize ghosts with their starting positions, colors, and states
    ghosts = [
        {"pos": [GHOST_SPAWN_X_COORDS[0], GHOST_SPAWN_Y], "color": GHOST_COLORS[0], "dir": [0,0], "id": 0, "is_eaten": False},
        {"pos": [GHOST_SPAWN_X_COORDS[1], GHOST_SPAWN_Y], "color": GHOST_COLORS[1], "dir": [0,0], "id": 1, "is_eaten": False},
        {"pos": [GHOST_SPAWN_X_COORDS[2], GHOST_SPAWN_Y], "color": GHOST_COLORS[2], "dir": [0,0], "id": 2, "is_eaten": False},
        {"pos": [GHOST_SPAWN_X_COORDS[3], GHOST_SPAWN_Y], "color": GHOST_COLORS[3], "dir": [0,0], "id": 3, "is_eaten": False},
    ]
    
    # More robust pellet initialization to avoid index errors
    pellets = [[False for _ in range(WIDTH)] for _ in range(HEIGHT)]
    for y in range(HEIGHT):
        for x in range(WIDTH):
            if y < len(maze) and x < len(maze[y]) and (maze[y][x] == '.' or maze[y][x] == 'o'):
                pellets[y][x] = True
    
    score = 0
    lives = 3
    power_timer = 0.0
    particles = []  # Reset particles
    # Vibe mode persists between games, so don't reset it
    # game_state is typically set to "playing" by the caller after reset, or "start" initially
    # running = True # Ensure game is set to run (usually true unless quit)

async def main():
    """Main asynchronous function to run the game, compatible with Pygbag."""
    global running, game_state # Allow modification of global running flag and game_state

    reset_game() # Initialize all game variables
    game_state = "start" # Begin at the start screen

    while running:
        update_loop() # Process events, update game logic, and draw the screen
        
        # The `running` flag is set to False by QUIT event in `update_loop`
        # Game state transitions (won/lost) are handled within `update_loop`'s drawing sections.
        # Player can restart from win/loss screens via key press.

        await asyncio.sleep(1.0 / FPS) # Control frame rate, yield control for web environment

    # Pygame quit is called when the `running` loop terminates
    pygame.quit()
    # sys.exit() # Optionally, ensure the script exits cleanly, especially for desktop

# --- Game Execution ---
if __name__ == "__main__":
    # Check if running in a web environment (Pygbag/Emscripten/Pyodide) or standard Python
    if platform.system() == "Emscripten" or "pyodide" in sys.modules:
        asyncio.run(main()) # Use asyncio.run for web deployment
    else: # Standard desktop Python execution
        # For desktop, asyncio might be overkill if not strictly needed,
        # but using it consistently for main loop structure.
        asyncio.run(main())
