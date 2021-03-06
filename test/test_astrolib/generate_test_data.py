#!/usr/bin/env python3

# Generate data for testing the AstroLib code

import sys
from math import pi, floor
from random import random
import ephem
from jinja2 import Template
from ephem.stars import stars as ephem_stars

tmpl = Template('''/* Test data generated by running '{{command}}'. DO NOT EDIT! */

#include "AstroLib.h"

struct TestAngle {
    float rad;
    SxAngle sx;
};

struct TestDate {
    const char *date;
    float jd;      // Julian date
    float gmst;    // Greenwich mean sidereal time
    float lst;     // Local sidereal time
};

struct TestStar {
    const char *name;
    EqCoords eq;
    HorizCoords hc;
};

TestAngle test_angles[] = {
{%- for a in angles %}
    { {{ a['rad'] | round(8)}}, { {{a['deg']}}, {{a['min']}}, {{a['sec']}}, {{a['sign']}} } },
{%- endfor %}
};

TestDate test_dates[] = {
{%- for d in dates %}
    { "{{ d['str']}}", {{d['jd'] | round(6)}}, {{d['gmst'] | round(6)}}, {{d['lst'] | round(6)}} },
{%- endfor %}
};

const char *obs_date = "{{obs.date}}";
Location obs_loc = { {{obs.lon | float}}, {{obs.lat | float}} };

TestStar test_stars[] = {
{%- for s in stars %}
    { "{{ s.name}}", { {{s.ra | round(6)}}, {{s.dec | round(6)}} }, { {{s.az | round(6)}}, {{s.alt | round(6)}} {{'}}'}},
{%- endfor %}
};
''')

def rad2sx(a):
    deg = abs(a*180/pi)
    min = 60*(deg % 1)
    sec = 60*(min % 1)
    return int(floor(deg)), int(floor(min)), int(round(sec))


def gen_test_angles():
    """Generate test angles"""
    test_angles = [2*pi*random() for i in range(20)]
    test_angles += [-2*pi*random() for i in range(20)]
    angles = []
    for a in test_angles:
        deg, min, sec = rad2sx(a)
        angle = {
            'rad': a,
            'deg': deg,
            'min': min,
            'sec': sec,
            'sign': int(a < 0),
        }
        angles.append(angle)
    return angles


def gen_test_dates(obs):
    """Generate test dates"""
    greenwich = ephem.Observer()
    greenwich.lat = '51:28:38'

    str_dates = [
        '1978/11/13 04:34:00',
        '1984/05/30 02:20:00',
        '2000/01/01 00:00:00',
        '2003/05/06 08:03:01',
        '2004/02/29 12:59:11',
        '2014/12/31 16:10:22',
        '2017/12/11 22:02:33',
        '2037/05/11 20:01:44',
    ]

    dates = []
    for str_date in str_dates:
        greenwich.date = str_date
        obs.date = str_date
        date = {
            'str': str_date,
            'jd': ephem.julian_date(str_date),
            'gmst': greenwich.sidereal_time(),
            'lst': obs.sidereal_time(),
        }
        dates.append(date)
    return dates


def gen_test_stars(obs):
    stars = []
    for name, star in ephem_stars.items():
        star.compute(obs)
        stars.append(star)
    return stars


if __name__ == "__main__":
    obs = ephem.city('Madrid')
    obs.pressure = 0  # ignore the effect of refraction

    angles = gen_test_angles()
    dates = gen_test_dates(obs)

    obs.date = '2020/9/12 01:50:41'
    stars = gen_test_stars(obs)

    txt = tmpl.render(command=sys.argv[0], angles=angles, dates=dates,
                      stars=stars, obs=obs)
    open('test_data.h', 'w').write(txt)
