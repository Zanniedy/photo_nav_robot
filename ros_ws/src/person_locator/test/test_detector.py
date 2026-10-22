import math

import numpy as np
import pytest

from person_locator.detector import decode_person, dedup

LAYOUT = []
for s, hw in ((8, 80), (16, 40), (32, 20)):
    LAYOUT += [{'name': f'box_s{s}', 'kind': 'box', 'stride': s, 'channels': 4},
               {'name': f'cls_s{s}', 'kind': 'cls', 'stride': s, 'channels': 80},
               {'name': f'mc_s{s}', 'kind': 'mc', 'stride': s, 'channels': 32}]
LAYOUT.append({'name': 'proto', 'kind': 'proto', 'stride': 4, 'channels': 32})


def outputs(person_at=None, other_at=None, nhwc=False):
    outs = []
    for meta in LAYOUT:
        s = meta['stride']
        hw = 640 // s if meta['kind'] != 'proto' else 160
        a = np.full((1, meta['channels'], hw, hw), -10.0, np.float32) if meta['kind'] == 'cls' \
            else np.ones((1, meta['channels'], hw, hw), np.float32)
        if meta['kind'] == 'cls' and s == 16:
            if person_at:
                a[0, 0, person_at[1], person_at[0]] = 2.0          # sigmoid(2)=0.88
            if other_at:
                a[0, 5, other_at[1], other_at[0]] = 3.0            # 别的类不该出现
        if meta['kind'] == 'box' and s == 16 and person_at:
            a[0, :, person_at[1], person_at[0]] = [2, 3, 4, 5]    # l t r b（格）
        outs.append(a.transpose(0, 2, 3, 1) if nhwc else a)
    return outs


@pytest.mark.parametrize('nhwc', [False, True])
def test_decode_single_person(nhwc):
    d = decode_person(outputs((10, 20), (5, 5), nhwc), LAYOUT, conf=0.5, nhwc=nhwc)
    assert d.shape == (1, 5)
    gx, gy = 10.5, 20.5
    assert d[0, :4] == pytest.approx(np.array([gx - 2, gy - 3, gx + 4, gy + 5]) * 16)
    assert d[0, 4] == pytest.approx(1 / (1 + math.exp(-2)), abs=1e-5)


def test_decode_nothing():
    assert decode_person(outputs(), LAYOUT, conf=0.5).shape == (0, 5)


def test_dedup():
    d = np.array([[0, 0, 10, 10, 0.9], [1, 1, 10, 10, 0.8], [20, 20, 30, 30, 0.7]], np.float32)
    assert len(dedup(d)) == 2
