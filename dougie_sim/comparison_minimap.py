"""Small evaluator-only route/coverage overlays, independent of video frame rate."""
import math

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .survey_comparison import AVAILABLE, RADIATION, BUDGET


class ComparisonMinimap:
    width, map_height, padding = 180, 90, 8
    unseen_color = (54, 69, 85)
    explored_color = (54, 125, 100)
    background_color = (11, 23, 36)
    available_color = (67, 149, 222)
    radiation_color = (222, 76, 89)
    budget_color = (202, 151, 67)

    def __init__(self, points, world, start, awareness='aware'):
        self.points = np.asarray(points)
        self.world, self.start = world, np.asarray(start)
        # Split connector surfaces as well as overlapping storeys. These are
        # display bands only; the existing coverage denominator is unchanged.
        heights, counts = np.unique(np.round(self.points[:, 2]/.02)*.02, return_counts=True)
        # Broad horizontal surfaces dominate a height bin. Thin, evenly spaced
        # ramp cross-sections must not become spurious floors.
        dominant = heights[counts >= max(32, .01*len(points))]
        self.heights = [float(self.points[:, 2].min())]
        if world != 'reactor':
            for height in dominant:
                if height-self.heights[-1] >= .3:
                    self.heights.append(float(height))
        self.boundaries = (np.asarray(self.heights[1:])+self.heights[:-1])/2
        self.levels = tuple(f'Z {height:.2f} m' for height in self.heights)
        self.layers = np.searchsorted(self.boundaries, self.points[:, 2], side='right')
        self.route_color = (255, 180, 84) if awareness == 'blind' else (72, 218, 202)
        self.lower, self.upper = self.points[:, :2].min(axis=0), self.points[:, :2].max(axis=0)
        span = np.maximum(self.upper-self.lower, .25)
        self.scale = min((self.width-2*self.padding)/span[0],
                         (self.map_height-2*self.padding)/span[1])
        self.centre = (self.lower+self.upper)/2
        self.pixels = self.project(self.points[:, :2])
        self.radius = max(1, int(math.ceil(.125*self.scale)))
        self.routes = [Image.new('L', (self.width, self.map_height)) for _ in self.levels]
        self.route_pens = [ImageDraw.Draw(image) for image in self.routes]
        self.last_position = None
        self.last_height = None
        self.backgrounds = [Image.new('RGB', (self.width, self.map_height), self.background_color)
                            for _ in self.levels]
        self.pens = [ImageDraw.Draw(image) for image in self.backgrounds]
        for point, layer in zip(self.pixels, self.layers):
            self._cell(point, layer, self.unseen_color)
        self.displayed_seen = np.zeros(len(points), dtype=bool)
        self.displayed_status = np.zeros(len(points), dtype=np.uint8)
        self.font = ImageFont.load_default()

    def project(self, xy):
        xy = np.asarray(xy)
        return (xy-self.centre)*[self.scale, -self.scale]+[self.width/2, self.map_height/2]

    def layer(self, support_height):
        return int(np.searchsorted(self.boundaries, support_height, side='right'))

    def _cell(self, point, layer, color):
        x, y = point
        r = self.radius
        self.pens[layer].rectangle((x-r, y-r, x+r, y+r), fill=color)

    def record(self, position, support_height):
        """Called on physics ticks, so skipped video frames never skip turns."""
        position = np.asarray(position)[:2].copy()
        if self.last_position is not None:
            first, last = self.project(self.last_position), self.project(position)
            fractions = [0., 1.]
            if support_height != self.last_height:
                fractions += [float((boundary-self.last_height)/(support_height-self.last_height))
                              for boundary in self.boundaries
                              if min(self.last_height, support_height) < boundary < max(self.last_height, support_height)]
            fractions.sort()
            for a, b in zip(fractions, fractions[1:]):
                layer = self.layer(self.last_height+(support_height-self.last_height)*(a+b)/2)
                self.route_pens[layer].line([tuple(first+a*(last-first)), tuple(first+b*(last-first))], fill=255, width=2)
        self.last_position, self.last_height = position, support_height

    def route_masks(self):
        return np.stack([np.asarray(image) for image in self.routes])

    def draw(self, seen, routes, position, support_height, yaw, admission=None):
        seen = np.asarray(seen, dtype=bool)
        admission = np.zeros(len(seen), dtype=np.uint8) if admission is None else np.asarray(admission)
        changed = (seen != self.displayed_seen) | (admission != self.displayed_status)
        for index in np.flatnonzero(changed):
            status = admission[index]
            color = (self.radiation_color if status == RADIATION else self.budget_color if status == BUDGET
                     else self.explored_color if seen[index] else self.available_color if status == AVAILABLE
                     else self.unseen_color)
            self._cell(self.pixels[index], self.layers[index], color)
        self.displayed_seen |= seen
        self.displayed_status = admission.copy()
        current_layer = self.layer(support_height)
        heights = [self.map_height if layer == current_layer else 42 for layer in range(len(self.levels))]
        height = 23+sum(h+16 for h in heights)+43
        result = Image.new('RGB', (self.width, height), self.background_color)
        draw = ImageDraw.Draw(result)
        percent = math.floor(1000.*seen.mean())/10
        draw.text((6, 5), f'{self.world.upper()}  {percent:.1f}%', font=self.font, fill='white')
        top = 23
        for layer, title in enumerate(self.levels):
            mask = self.layers == layer
            percent = math.floor(1000.*seen[mask].mean())/10 if mask.any() else 0.
            label = title+(' *' if layer == current_layer else '')+f'  {percent:.1f}%'
            draw.text((6, top), label, font=self.font, fill='#b8c8d8')
            image = self.backgrounds[layer].copy()
            image.paste(self.route_color, mask=Image.fromarray(routes[layer].astype(np.uint8)))
            pen = ImageDraw.Draw(image)
            if layer == self.layer(self.start[2]):
                x, y = self.project(self.start[:2])
                pen.rectangle((x-3, y-3, x+3, y+3), outline='white', width=1)
            if layer == current_layer:
                centre = self.project(np.asarray(position)[:2])
                forward = np.array([math.cos(yaw), -math.sin(yaw)])
                side = np.array([-forward[1], forward[0]])
                arrow = [centre+forward*6, centre-forward*4+side*4, centre-forward*4-side*4]
                pen.polygon([tuple(p) for p in arrow], fill='white', outline='#08121d')
            if heights[layer] != self.map_height:
                image = image.resize((round(self.width*heights[layer]/self.map_height), heights[layer]),
                                     getattr(Image, 'Resampling', Image).NEAREST)
            result.paste(image, ((self.width-image.width)//2, top+14))
            top += heights[layer]+16
        draw = ImageDraw.Draw(result)
        y = height-41
        draw.rectangle((6, y+2, 12, y+8), fill=self.explored_color)
        draw.text((16, y), 'Explored', font=self.font, fill='#b8c8d8')
        draw.line((90, y+5, 101, y+5), fill=self.route_color, width=2)
        draw.text((105, y), 'Route', font=self.font, fill='#b8c8d8')
        for x, color, label in ((6, self.available_color, 'Can go'), (66, self.radiation_color, 'Risk'),
                                (119, self.budget_color, 'Budget')):
            draw.rectangle((x, y+16, x+5, y+21), fill=color)
            draw.text((x+8, y+13), label, font=self.font, fill='#b8c8d8')
        draw.text((6, height-12), 'Square: start  Arrow: robot', font=self.font, fill='#b8c8d8')
        draw.rectangle((0, 0, self.width-1, height-1), outline='#53697f')
        return result
