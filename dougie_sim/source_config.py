"""Validated, simulator-only point-source scenarios; all coordinates in metres."""
import json
import math
from pathlib import Path
import re

DEFAULT_SOURCE_CONFIG = Path(__file__).resolve().parents[1]/'config/dougie_radiation_sources.json'


def number(value, name, minimum=0.):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < minimum:
        raise ValueError(f'{name} must be a finite number >= {minimum}')
    return float(value)


def load_source_profile(profile='baseline', path=None):
    path = Path(path) if path is not None else DEFAULT_SOURCE_CONFIG
    document = json.loads(path.read_text())
    if not isinstance(document, dict) or document.get('schema_version') != 1:
        raise ValueError('unsupported radiation source schema')
    catalogue, profiles = document.get('sources'), document.get('profiles')
    if not isinstance(catalogue, dict) or not isinstance(profiles, dict):
        raise ValueError('source catalogue and profiles must be objects')
    if profile not in profiles:
        raise ValueError(f'unknown radiation profile {profile!r}; choose from {", ".join(profiles)}')
    selected = profiles[profile]
    if not isinstance(selected, dict) or set(selected) != {'background_usv_h', 'sources'}:
        raise ValueError('each profile needs background_usv_h and sources')
    ids = selected['sources']
    if not isinstance(ids, list) or not all(isinstance(key, str) for key in ids) or len(ids) > 64 or len(set(ids)) != len(ids):
        raise ValueError('profile must list at most 64 unique source IDs')
    result = dict(background_usv_h=number(selected['background_usv_h'], 'background'), sources=[])
    for identifier in ids:
        if not re.fullmatch(r'[a-z][a-z0-9_]{0,63}', identifier) or identifier not in catalogue:
            raise ValueError(f'invalid or missing source ID: {identifier}')
        source = catalogue[identifier]
        required = {'kind', 'position', 'strength_usv_h_m2', 'core_radius_m'}
        if not isinstance(source, dict) or not required <= source.keys() or source.keys()-required-{'enabled'}:
            raise ValueError(f'invalid fields for source {identifier}')
        if source['kind'] not in ('hotspot', 'leak') or not isinstance(source.get('enabled', True), bool):
            raise ValueError(f'invalid kind/enabled value for source {identifier}')
        position = source['position']
        if not isinstance(position, list) or len(position) != 3:
            raise ValueError(f'{identifier}.position must be [x, y, z]')
        position = [number(v, f'{identifier}.position', minimum=-math.inf) for v in position]
        strength = number(source['strength_usv_h_m2'], identifier+'.strength_usv_h_m2')
        radius = number(source['core_radius_m'], identifier+'.core_radius_m', minimum=.001)
        if not math.isfinite(radius*radius) or not math.isfinite(strength/(radius*radius)):
            raise ValueError(f'{identifier}: nonfinite peak rate')
        if source.get('enabled', True):
            result['sources'].append(dict(id=identifier, kind=source['kind'], position=position,
                strength_usv_h_m2=strength, core_radius_m=radius))
    return result
