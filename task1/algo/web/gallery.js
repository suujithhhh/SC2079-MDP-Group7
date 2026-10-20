/**
 * Camera gallery helpers for the display-only page (`PROTOCOL.md` section 7.1).
 *
 * No DOM access and no `fetch`: pure functions, imported by `auto.js` in the
 * browser and by Node's test runner (`tests/gallery.test.js`).
 *
 * The RPi uploads one photo per obstacle after each SNAP; the PC lists them at
 * `GET /images`. Rules here:
 * - a malformed list is ignored entirely (fail closed): nothing is shown;
 * - only photos whose `mission_id` equals the mission on screen are shown, so
 *   an earlier run's photos disappear when a new plan starts;
 * - an image URL is built only from an `image_id` of the exact form the PC
 *   issues, never from anything else in the reply;
 * - `target_id` is the image ID the RPi reported (or `null`: none reported). It
 *   is shown as given, never checked against the picture, and must be an
 *   integer from 11 to 40 or the whole list is ignored.
 */

export const IMAGE_ID_RE = /^[0-9]{1,2}-[0-9]{1,12}$/;
export const MAX_GALLERY_IMAGES = 8;
export const TARGET_ID_MIN = 11;
export const TARGET_ID_MAX = 40;

const isFiniteNumber = (v) => typeof v === 'number' && Number.isFinite(v);

/**
 * Validates and normalizes a `GET /images` body. Returns
 * `{ missionId, revision, images: [{ imageId, obstacleId, targetId, bytes, receivedAtS }] }`,
 * or `null` when anything is off (wrong type, bad id, duplicate obstacle, too
 * many photos, a photo with no mission, a `target_id` outside 11-40, ...).
 */
export function normalizeImageList(raw) {
  if (!raw || typeof raw !== 'object' || raw.type !== 'IMAGE_LIST') return null;
  if (raw.mission_id !== null && (typeof raw.mission_id !== 'string' || raw.mission_id === '')) return null;
  if (!Number.isInteger(raw.revision) || raw.revision < 0) return null;
  if (!Array.isArray(raw.images) || raw.images.length > MAX_GALLERY_IMAGES) return null;
  if (raw.images.length > 0 && raw.mission_id === null) return null;

  const seenObstacles = new Set();
  const seenIds = new Set();
  const images = [];
  for (const item of raw.images) {
    if (!item || typeof item !== 'object') return null;
    if (typeof item.image_id !== 'string' || !IMAGE_ID_RE.test(item.image_id)) return null;
    if (!Number.isInteger(item.obstacle_id) || item.obstacle_id < 1 || item.obstacle_id > 8) return null;
    const targetId = item.target_id === undefined ? null : item.target_id;
    if (targetId !== null
        && (!Number.isInteger(targetId) || targetId < TARGET_ID_MIN || targetId > TARGET_ID_MAX)) return null;
    if (!isFiniteNumber(item.bytes) || item.bytes < 0 || !isFiniteNumber(item.received_at_s)) return null;
    if (seenObstacles.has(item.obstacle_id) || seenIds.has(item.image_id)) return null;
    seenObstacles.add(item.obstacle_id);
    seenIds.add(item.image_id);
    images.push({
      imageId: item.image_id,
      obstacleId: item.obstacle_id,
      targetId,
      bytes: item.bytes,
      receivedAtS: item.received_at_s,
    });
  }
  return { missionId: raw.mission_id, revision: raw.revision, images };
}

/**
 * The photos to show for the mission on screen: none unless the list is valid
 * and its mission equals `missionId`. Oldest first, i.e. in the order the
 * robot snapped them.
 */
export function selectGalleryImages(list, missionId) {
  if (!list || typeof missionId !== 'string' || missionId === '' || list.missionId !== missionId) return [];
  return [...list.images].sort((a, b) => a.receivedAtS - b.receivedAtS || a.obstacleId - b.obstacleId);
}

/** `/images/<image_id>`; throws on an id the PC would never issue. */
export function imageUrl(image) {
  if (!image || typeof image.imageId !== 'string' || !IMAGE_ID_RE.test(image.imageId)) {
    throw new RangeError('not a valid image id');
  }
  return `/images/${image.imageId}`;
}

/** Changes exactly when the set of tiles on screen must be rebuilt (a replaced
 * photo gets a new `image_id`, so a retake, or a re-upload that adds the
 * `target_id`, changes the key). */
export function galleryKey(images, missionId) {
  return `${missionId ?? ''}|${images.map((image) => image.imageId).join(',')}`;
}

/** Short caption text for a tile; plain text, never HTML. "Obstacle 3" or,
 * when the RPi reported an image ID, "Obstacle 3 \u00b7 ID 22". */
export function tileCaption(image) {
  const base = `Obstacle ${image.obstacleId}`;
  return Number.isInteger(image.targetId) ? `${base} \u00b7 ID ${image.targetId}` : base;
}

/** Heading counter: "" when empty, else "3 photos" / "1 photo". */
export function countText(images) {
  if (images.length === 0) return '';
  return `${images.length} photo${images.length === 1 ? '' : 's'}`;
}
