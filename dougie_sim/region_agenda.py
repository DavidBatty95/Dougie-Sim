"""Observed connected-region scheduling above fine-graph route admission.

Spatial bins bound region extent; disconnected parts never share a region. These
are navigation regions, not semantic rooms or supplied floor labels.
"""
from collections import Counter
import math


class RegionAgenda:
    def __init__(self):
        self.membership = {}
        self.transitions = Counter()
        self.current = None
        self.last_served = {}
        self.pending = {}
        self.connectors = set()
        self.visit_fraction = {}

    def update(self, nodes, graph, root, stamp, visited=()):
        tags = {k: (math.floor(p[0]/3.), math.floor(p[1]/3.), math.floor((p[2]+.1)/.4))
                for k, p in nodes.items()}
        # Weak connectivity for region grouping; fine directed route admission
        # remains authoritative for both outward travel and return.
        adjacent = {k: set() for k in nodes}
        for a, neighbours in graph.items():
            for b in neighbours:
                if a in nodes and b in nodes:
                    adjacent[a].add(b)
                    adjacent[b].add(a)
        membership = {}
        for seed in sorted(nodes):
            if seed in membership:
                continue
            region = (tags[seed], seed)
            membership[seed] = region
            queue = [seed]
            for a in queue:
                for b in sorted(adjacent[a]):
                    if b not in membership and tags[b] == tags[seed]:
                        membership[b] = region
                        queue.append(b)
        self.membership = membership
        sizes = Counter(membership.values())
        visits = Counter(membership[k] for k in visited if k in membership)
        self.visit_fraction = {region: visits[region]/size for region, size in sizes.items()}
        regions = set(membership.values())
        self.transitions = Counter({k: v for k, v in self.transitions.items() if k[0] in regions and k[1] in regions})
        self.last_served = {k: v for k, v in self.last_served.items() if k in regions}
        current = membership.get(root)
        if self.current in regions and current is not None and self.current != current:
            self.transitions[self.current, current] += 1
        self.current = current
        if current is not None:
            self.last_served[current] = stamp
        self.connectors = {(membership[a], membership[b]) for a, neighbours in graph.items()
                           for b in neighbours if a in membership and b in membership and membership[a] != membership[b]}

    def choose(self, candidates, duration, stamp, inspection_keys=()):
        groups = {}
        for item in candidates:
            groups.setdefault(self.membership[item[1]], []).append(item)
        self.pending = {region: len(items) for region, items in groups.items()}
        def priority(region):
            items = groups[region]
            best = max(items)
            # Prefer finishing useful local work, but allow a sufficiently more
            # valuable distant region to win. Aging prevents a hard local cutoff.
            age = min(1., max(0., stamp-self.last_served.get(region, 0.))/120.)
            repeats = self.transitions[self.current, region]
            utility = best[0]*(1.+.25*age)
            # Repeated near-frontier views on a traversed ramp must not dominate
            # the unvisited landing it connects to. Saturate after half the
            # supported centres have been covered by the robot footprint.
            # A visited region can still contain a newly observed inspection
            # target. Do not suppress that new semantic work as a spatial revisit.
            if not any(item[1] in inspection_keys for item in items):
                utility *= max(.2, 1.-1.6*self.visit_fraction.get(region, 0.))
            if region == self.current:
                utility *= 1.15
            return utility/(1.+.15*repeats+.002*min(duration[item[1]] for item in items)), region
        region = max(groups, key=priority)
        return max(groups[region])

    def metrics(self):
        return dict(regions=len(set(self.membership.values())), connectors=len(self.connectors),
                    pending_regions=len(self.pending), connector_crossings=sum(self.transitions.values()),
                    repeated_connector_crossings=sum(max(0, n-1) for n in self.transitions.values()))
