// Tests for algo/web/gallery.js: the camera gallery's pure rules.
//
// Run with:  node --test algo/web/tests/gallery.test.js

import { test, describe } from 'node:test';
import assert from 'node:assert/strict';

import {
  IMAGE_ID_RE,
  countText,
  galleryKey,
  imageUrl,
  normalizeImageList,
  selectGalleryImages,
  tileCaption,
} from '../gallery.js';

const item = (obstacleId, seq, at = 100, targetId = null) => ({
  image_id: `${obstacleId}-${seq}`, obstacle_id: obstacleId, target_id: targetId, bytes: 1000 + obstacleId,
  received_at_s: at,
});
const reply = (images, extra = {}) => ({
  type: 'IMAGE_LIST', mission_id: 'm1', revision: images.length, images, ...extra,
});

describe('normalizeImageList', () => {
  test('a valid list is normalized to camelCase fields', () => {
    const list = normalizeImageList(reply([item(3, 1, 10), item(5, 2, 20)]));
    assert.equal(list.missionId, 'm1');
    assert.equal(list.revision, 2);
    assert.deepEqual(list.images[0], { imageId: '3-1', obstacleId: 3, targetId: null, bytes: 1003, receivedAtS: 10 });
  });

  test('a target ID from 11 to 40 is kept; a missing one is null', () => {
    const list = normalizeImageList(reply([item(3, 1, 10, 22), item(5, 2, 20, 11), item(6, 3, 30, 40)]));
    assert.deepEqual(list.images.map((i) => i.targetId), [22, 11, 40]);
    const { target_id: _omitted, ...withoutKey } = item(2, 4);
    assert.equal(normalizeImageList(reply([withoutKey])).images[0].targetId, null);
  });

  test('a target ID outside 11-40 or of the wrong type rejects the whole list', () => {
    for (const targetId of [0, 10, 41, 99, -22, 22.5, '22', true, [22], {}, NaN]) {
      assert.equal(normalizeImageList(reply([item(3, 1, 10, targetId)])), null, String(targetId));
    }
  });

  test('an empty list is valid, with or without a mission', () => {
    assert.deepEqual(normalizeImageList({ type: 'IMAGE_LIST', mission_id: null, revision: 0, images: [] }).images, []);
    assert.equal(normalizeImageList(reply([])).missionId, 'm1');
  });

  test('anything malformed is rejected whole (fail closed)', () => {
    const bad = [
      null, undefined, 'x', 5, [], {},
      { ...reply([]), type: 'PLAN_RESULT' },
      reply([], { mission_id: '' }),
      reply([], { mission_id: 7 }),
      reply([], { revision: -1 }),
      reply([], { revision: 1.5 }),
      reply([], { revision: '1' }),
      reply('nope'),
      reply([item(1, 1)], { mission_id: null }),
      reply([item(1, 1), item(1, 2)]),
      reply([item(1, 1), item(2, 1)].map((i) => ({ ...i, image_id: '1-1' }))),
      reply([{ ...item(1, 1), image_id: '../x' }]),
      reply([{ ...item(1, 1), image_id: '1-1/..' }]),
      reply([{ ...item(1, 1), image_id: 5 }]),
      reply([{ ...item(0, 1), obstacle_id: 0 }]),
      reply([{ ...item(9, 1), obstacle_id: 9 }]),
      reply([{ ...item(1, 1), obstacle_id: '1' }]),
      reply([{ ...item(1, 1), obstacle_id: 1.5 }]),
      reply([{ ...item(1, 1), bytes: -1 }]),
      reply([{ ...item(1, 1), bytes: null }]),
      reply([{ ...item(1, 1), received_at_s: 'now' }]),
      reply([{ ...item(1, 1), received_at_s: Infinity }]),
      reply([null]),
      reply(Array.from({ length: 9 }, (_, i) => item(1 + (i % 8), i + 1))),
    ];
    for (const raw of bad) assert.equal(normalizeImageList(raw), null, JSON.stringify(raw)?.slice(0, 80));
  });
});

describe('selectGalleryImages', () => {
  const list = normalizeImageList(reply([item(5, 3, 30), item(2, 1, 10), item(7, 2, 20)]));

  test('only the mission on screen is shown, oldest first', () => {
    assert.deepEqual(selectGalleryImages(list, 'm1').map((i) => i.obstacleId), [2, 7, 5]);
  });

  test('another mission, no mission, or no list shows nothing', () => {
    assert.deepEqual(selectGalleryImages(list, 'm2'), []);
    assert.deepEqual(selectGalleryImages(list, null), []);
    assert.deepEqual(selectGalleryImages(list, ''), []);
    assert.deepEqual(selectGalleryImages(null, 'm1'), []);
    assert.deepEqual(selectGalleryImages(list, 5), []);
  });

  test('ties in time fall back to the obstacle number, and the input is never mutated', () => {
    const tied = normalizeImageList(reply([item(6, 1, 5), item(2, 2, 5)]));
    const before = JSON.stringify(tied);
    assert.deepEqual(selectGalleryImages(tied, 'm1').map((i) => i.obstacleId), [2, 6]);
    assert.equal(JSON.stringify(tied), before);
  });
});

describe('imageUrl, galleryKey, captions', () => {
  test('a URL is built only from an id of the exact form the PC issues', () => {
    assert.equal(imageUrl({ imageId: '3-17' }), '/images/3-17');
    for (const imageId of ['../x', '3-', '-3', '3-1/a', '3-1?x=1', 'http://evil/1-1', '', null, undefined, 5, '100-1']) {
      assert.throws(() => imageUrl({ imageId }), RangeError, String(imageId));
    }
    assert.throws(() => imageUrl(null), RangeError);
    assert.ok(IMAGE_ID_RE.test('8-123456789012'));
  });

  test('the key changes with the mission, the set of photos and a retake, but not with the byte count', () => {
    const a = [{ imageId: '1-1', obstacleId: 1, bytes: 10 }];
    assert.equal(galleryKey(a, 'm1'), galleryKey([{ ...a[0], bytes: 99 }], 'm1'));
    assert.notEqual(galleryKey(a, 'm1'), galleryKey(a, 'm2'));
    assert.notEqual(galleryKey(a, 'm1'), galleryKey([{ imageId: '1-2', obstacleId: 1 }], 'm1'));
    assert.notEqual(galleryKey(a, 'm1'), galleryKey([], 'm1'));
    assert.equal(galleryKey([], null), '|');
  });

  test('captions and the counter are plain text', () => {
    assert.equal(tileCaption({ obstacleId: 4 }), 'Obstacle 4');
    assert.equal(tileCaption({ obstacleId: 4, targetId: null }), 'Obstacle 4');
    assert.equal(tileCaption({ obstacleId: 3, targetId: 22 }), 'Obstacle 3 \u00b7 ID 22');
    assert.equal(countText([]), '');
    assert.equal(countText([{}]), '1 photo');
    assert.equal(countText([{}, {}, {}]), '3 photos');
  });
});
