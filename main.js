const canvas = document.getElementById('scene');
const ctx = canvas.getContext('2d');
const sidesEl = document.getElementById('sidesVal');
const speedEl = document.getElementById('speedVal');

const config = {
  initialSides: 3,
  ballRadius: 8,
  initialSpeed: 220, // px/s
  speedGainOnBounce: 1.06,
  maxSpeed: 2500,
  polygonRadiusRatio: 0.42, // of min(canvasWidth, canvasHeight)
  polygonStroke: '#6bb2ff',
  polygonFill: 'rgba(107, 178, 255, 0.06)',
  ballFill: '#f2d27e',
  trail: {
    enabled: true,
    length: 40,
    fade: 0.9,
    color: 'rgba(242,210,126,0.75)'
  }
};

let deviceScale = 1;
let viewportWidth = 0;
let viewportHeight = 0;

function resizeCanvas() {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  deviceScale = dpr;
  viewportWidth = canvas.clientWidth;
  viewportHeight = canvas.clientHeight;
  canvas.width = Math.floor(viewportWidth * dpr);
  canvas.height = Math.floor(viewportHeight * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

window.addEventListener('resize', resizeCanvas);
resizeCanvas();

function regularPolygon(center, radius, sides, startAngle = -Math.PI/2) {
  const vertices = [];
  const step = (Math.PI * 2) / sides;
  for (let i = 0; i < sides; i++) {
    const angle = startAngle + i * step;
    vertices.push({
      x: center.x + radius * Math.cos(angle),
      y: center.y + radius * Math.sin(angle)
    });
  }
  return vertices;
}

function buildEdges(vertices) {
  const edges = [];
  const n = vertices.length;
  for (let i = 0; i < n; i++) {
    const a = vertices[i];
    const b = vertices[(i + 1) % n];
    const ex = b.x - a.x;
    const ey = b.y - a.y;
    const len = Math.hypot(ex, ey) || 1;
    const tx = ex / len; // tangent
    const ty = ey / len;
    const nx = -ty; // inward normal for CCW polygon
    const ny = tx;
    edges.push({ a, b, tx, ty, nx, ny, len });
  }
  return edges;
}

function pointToEdgeSignedDistance(p, edge) {
  // Signed distance positive when point is on inward side (left of edge vector)
  const vx = p.x - edge.a.x;
  const vy = p.y - edge.a.y;
  return vx * edge.nx + vy * edge.ny; // dot with inward normal
}

function projectPointOnSegment(p, a, b) {
  const abx = b.x - a.x;
  const aby = b.y - a.y;
  const apx = p.x - a.x;
  const apy = p.y - a.y;
  const abLen2 = abx * abx + aby * aby;
  const t = abLen2 === 0 ? 0 : Math.max(0, Math.min(1, (apx * abx + apy * aby) / abLen2));
  return { x: a.x + abx * t, y: a.y + aby * t, t };
}

function reflectVelocity(vx, vy, nx, ny) {
  const vDotN = vx * nx + vy * ny;
  const rx = vx - 2 * vDotN * nx;
  const ry = vy - 2 * vDotN * ny;
  return { vx: rx, vy: ry };
}

function clampMagnitude(vx, vy, max) {
  const m = Math.hypot(vx, vy);
  if (m <= max) return { vx, vy };
  const s = max / (m || 1);
  return { vx: vx * s, vy: vy * s };
}

class World {
  constructor() {
    this.center = { x: canvas.clientWidth / 2, y: canvas.clientHeight / 2 };
    this.radius = Math.min(canvas.clientWidth, canvas.clientHeight) * config.polygonRadiusRatio;
    this.sides = config.initialSides;
    this.startAngle = -Math.PI / 2; // flat base at bottom for triangle
    this.vertices = regularPolygon(this.center, this.radius, this.sides, this.startAngle);
    this.edges = buildEdges(this.vertices);
    this.ball = {
      x: this.center.x,
      y: this.center.y,
      r: config.ballRadius,
      vx: 0,
      vy: 0
    };
    // Initialize velocity with random direction
    const angle = Math.random() * Math.PI * 2;
    this.ball.vx = Math.cos(angle) * config.initialSpeed;
    this.ball.vy = Math.sin(angle) * config.initialSpeed;

    this.trail = [];
  }

  onResize() {
    this.center = { x: canvas.clientWidth / 2, y: canvas.clientHeight / 2 };
    this.radius = Math.min(canvas.clientWidth, canvas.clientHeight) * config.polygonRadiusRatio;
    this.vertices = regularPolygon(this.center, this.radius, this.sides, this.startAngle);
    this.edges = buildEdges(this.vertices);
  }

  addSide() {
    this.sides += 1;
    this.vertices = regularPolygon(this.center, this.radius, this.sides, this.startAngle);
    this.edges = buildEdges(this.vertices);
  }
}

const world = new World();
window.addEventListener('resize', () => world.onResize());

let lastTime = performance.now();

function tick(now) {
  const dt = Math.min((now - lastTime) / 1000, 0.033); // clamp huge frame gaps
  lastTime = now;
  physicsStep(dt);
  render();
  requestAnimationFrame(tick);
}

requestAnimationFrame(tick);

function physicsStep(dt) {
  const ball = world.ball;

  // Determine number of substeps to avoid tunneling
  const maxStepDist = Math.max(ball.r * 0.6, 2);
  const moveDist = Math.hypot(ball.vx, ball.vy) * dt;
  const steps = Math.max(1, Math.ceil(moveDist / maxStepDist));
  const subDt = dt / steps;

  for (let s = 0; s < steps; s++) {
    integrateAndCollide(subDt);
  }
}

function integrateAndCollide(dt) {
  const ball = world.ball;
  let nextX = ball.x + ball.vx * dt;
  let nextY = ball.y + ball.vy * dt;

  // Check against polygon edges for penetration at next position
  let collided = false;
  let bestEdge = null;
  let minPenetration = Infinity; // how deep inside the forbidden zone (radius boundary) we are

  for (const edge of world.edges) {
    // Signed distance from next pos to edge plane (inward positive)
    const d = pointToEdgeSignedDistance({ x: nextX, y: nextY }, edge) - ball.r;
    if (d < 0) {
      // Potential penetration. Confirm the closest point lies on the segment span
      const closest = projectPointOnSegment({ x: nextX, y: nextY }, edge.a, edge.b);
      const distToSeg = Math.hypot(nextX - closest.x, nextY - closest.y);
      // If distance to the infinite line equals distance to segment, projection lies within [0,1]
      if (closest.t >= 0 && closest.t <= 1 && distToSeg - ball.r <= 0.5 /* tolerance */) {
        if (d < minPenetration) {
          minPenetration = d; // most negative is deepest
          bestEdge = edge;
          collided = true;
        }
      }
    }
  }

  if (collided && bestEdge) {
    // Move ball out of penetration along inward normal
    const nx = bestEdge.nx;
    const ny = bestEdge.ny;

    nextX -= minPenetration * nx; // minPenetration is negative
    nextY -= minPenetration * ny;

    // Reflect velocity only if we are moving toward the edge
    const approaching = (ball.vx * nx + ball.vy * ny) < 0;
    if (approaching) {
      const rv = reflectVelocity(ball.vx, ball.vy, nx, ny);
      // Speed up on bounce
      rv.vx *= config.speedGainOnBounce;
      rv.vy *= config.speedGainOnBounce;
      const clamped = clampMagnitude(rv.vx, rv.vy, config.maxSpeed);
      ball.vx = clamped.vx;
      ball.vy = clamped.vy;

      // Add side after a valid bounce
      world.addSide();
      sidesEl.textContent = String(world.sides);
    }
  }

  // Commit new position
  ball.x = nextX;
  ball.y = nextY;

  // Trail
  if (config.trail.enabled) {
    world.trail.push({ x: ball.x, y: ball.y, life: 1.0 });
    if (world.trail.length > config.trail.length) {
      world.trail.shift();
    }
    for (const p of world.trail) p.life *= config.trail.fade;
  }

  // HUD speed display (screen-space units per second)
  const spd = Math.hypot(ball.vx, ball.vy);
  speedEl.textContent = Math.round(spd).toString();
}

function render() {
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;

  ctx.clearRect(0, 0, w, h);

  // Draw polygon fill
  ctx.beginPath();
  const verts = world.vertices;
  if (verts.length) {
    ctx.moveTo(verts[0].x, verts[0].y);
    for (let i = 1; i < verts.length; i++) {
      ctx.lineTo(verts[i].x, verts[i].y);
    }
    ctx.closePath();
    ctx.fillStyle = config.polygonFill;
    ctx.fill();

    // Stroke
    ctx.lineWidth = 2;
    ctx.strokeStyle = config.polygonStroke;
    ctx.stroke();
  }

  // Draw trail
  if (config.trail.enabled && world.trail.length > 1) {
    ctx.beginPath();
    for (let i = 0; i < world.trail.length - 1; i++) {
      const p0 = world.trail[i];
      const p1 = world.trail[i + 1];
      ctx.moveTo(p0.x, p0.y);
      ctx.lineTo(p1.x, p1.y);
    }
    const grad = ctx.createLinearGradient(0, 0, 0, h);
    grad.addColorStop(0, config.trail.color);
    grad.addColorStop(1, 'rgba(242,210,126,0.05)');
    ctx.strokeStyle = grad;
    ctx.lineWidth = 2;
    ctx.stroke();
  }

  // Draw ball
  ctx.beginPath();
  ctx.arc(world.ball.x, world.ball.y, world.ball.r, 0, Math.PI * 2);
  ctx.fillStyle = config.ballFill;
  ctx.fill();
}
