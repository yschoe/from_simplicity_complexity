# from_simplicity_complexity

Forked version from Alex Alvarez's repo.

Example usage:

```
python boids.py  --width=800 --height=800 --maxlen=50 --delay=20 --num_boids=200
```

Here's the full description:

```
usage: boids.py [-h] [--maxlen MAXLEN] [--delay DELAY] [--width WIDTH] [--height HEIGHT]
                [--num_boids NUM_BOIDS] [--vis_range VIS_RANGE]

options:
  -h, --help            show this help message and exit
  --maxlen MAXLEN       max buffer length (default: 35)
  --delay DELAY         delay (default: 1)
  --width WIDTH         arena width (default: 1024)
  --height HEIGHT       arena height (default: 768)
  --num_boids NUM_BOIDS
                        number of boids (default: 100)
  --vis_range VIS_RANGE
                        vis_range (default: 75)
```
