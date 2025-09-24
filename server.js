import express from "express";
import Database from "better-sqlite3";

const app = express();
app.use(express.json());

const db = new Database("routes.db");

db.exec(`
CREATE TABLE IF NOT EXISTS drivers (
  id TEXT PRIMARY KEY,
  name TEXT,
  licenseType TEXT,
  availability INTEGER DEFAULT 1,
  currentRoute INTEGER
);

CREATE TABLE IF NOT EXISTS routes (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  startLocation TEXT,
  endLocation TEXT,
  distance REAL,
  estimatedTime TEXT,
  driverId TEXT,
  status TEXT DEFAULT 'unassigned',
  FOREIGN KEY(driverId) REFERENCES drivers(id)
);

CREATE TABLE IF NOT EXISTS driver_history (
  driverId TEXT,
  routeId INTEGER,
  FOREIGN KEY(driverId) REFERENCES drivers(id),
  FOREIGN KEY(routeId) REFERENCES routes(id)
);
`);

app.post("/routes", (req, res) => {
  const { startLocation, endLocation, distance, estimatedTime } = req.body;

  const driver = db
    .prepare(
      "SELECT * FROM drivers WHERE availability = 1 AND currentRoute IS NULL LIMIT 1"
    )
    .get();

  let status = "unassigned";
  let driverId = null;

  if (driver) {
    status = "assigned";
    driverId = driver.id;
    db.prepare(
      "UPDATE drivers SET currentRoute = ?, availability = 0 WHERE id = ?"
    ).run(null, driverId);
  }

  const result = db
    .prepare(
      "INSERT INTO routes (startLocation, endLocation, distance, estimatedTime, driverId, status) VALUES (?,?,?,?,?,?)"
    )
    .run(startLocation, endLocation, distance, estimatedTime, driverId, status);

  const route = db
    .prepare("SELECT * FROM routes WHERE id = ?")
    .get(result.lastInsertRowid);

  if (driverId) {
    db.prepare("UPDATE drivers SET currentRoute = ? WHERE id = ?").run(
      route.id,
      driverId
    );
  }

  res.status(201).json(route);
});

app.post("/drivers", (req, res) => {
  const { id, name, licenseType, availability } = req.body;
  try {
    db.prepare(
      "INSERT INTO drivers (id, name, licenseType, availability) VALUES (?,?,?,?)"
    ).run(id, name, licenseType, availability ? 1 : 0);

    let driver = db.prepare("SELECT * FROM drivers WHERE id = ?").get(id);

    if (driver.availability === 1) {
      const unassignedRoute = db
        .prepare("SELECT * FROM routes WHERE status = 'unassigned' LIMIT 1")
        .get();

      if (unassignedRoute) {
        db.prepare(
          "UPDATE routes SET driverId = ?, status = 'assigned' WHERE id = ?"
        ).run(driver.id, unassignedRoute.id);

        db.prepare(
          "UPDATE drivers SET currentRoute = ?, availability = 0 WHERE id = ?"
        ).run(unassignedRoute.id, driver.id);

        driver = db.prepare("SELECT * FROM drivers WHERE id = ?").get(id);
      }
    }

    res.status(201).json(driver);
  } catch (err) {
    res.status(500).json({ error: err.message });
  }
});

// ✅ Get all drivers
app.get("/drivers", (req, res) => {
  const drivers = db.prepare("SELECT * FROM drivers").all();
  res.json(drivers);
});

app.patch("/drivers/:id/availability", (req, res) => {
  const { id } = req.params;
  const { availability } = req.body;

  if (availability !== 0 && availability !== 1) {
    return res
      .status(400)
      .json({ error: "Availability must be 0 (false) or 1 (true)" });
  }

  const driver = db.prepare("SELECT * FROM drivers WHERE id = ?").get(id);
  if (!driver) {
    return res.status(404).json({ error: "Driver not found" });
  }

  if (availability === 0 && driver.currentRoute !== null) {
    return res
      .status(400)
      .json({ error: "Driver cannot go off duty while assigned to a route" });
  }

  db.prepare("UPDATE drivers SET availability = ? WHERE id = ?").run(
    availability,
    id
  );

  let updatedDriver = db.prepare("SELECT * FROM drivers WHERE id = ?").get(id);

  if (availability === 1 && updatedDriver.currentRoute == null) {
    const unassignedRoute = db
      .prepare("SELECT * FROM routes WHERE status = 'unassigned' LIMIT 1")
      .get();

    if (unassignedRoute) {
      db.prepare(
        "UPDATE routes SET driverId = ?, status = 'assigned' WHERE id = ?"
      ).run(updatedDriver.id, unassignedRoute.id);

      db.prepare(
        "UPDATE drivers SET currentRoute = ?, availability = 0 WHERE id = ?"
      ).run(unassignedRoute.id, updatedDriver.id);

      updatedDriver = db.prepare("SELECT * FROM drivers WHERE id = ?").get(id);
    }
  }

  res.json(updatedDriver);
});

app.get("/schedule", (req, res) => {
  const routes = db
    .prepare(
      `
    SELECT r.*, d.name as driverName, d.id as driverId
    FROM routes r
    LEFT JOIN drivers d ON r.driverId = d.id
  `
    )
    .all();
  res.json(routes);
});

app.get("/drivers/:id/history", (req, res) => {
  const { id } = req.params;
  const history = db
    .prepare(
      `
    SELECT r.* 
    FROM driver_history h
    JOIN routes r ON h.routeId = r.id
    WHERE h.driverId = ?
  `
    )
    .all(id);
  res.json(history);
});

app.get("/routes", (req, res) => {
  const page = parseInt(req.query.page) || 1;
  const limit = parseInt(req.query.limit) || 5;
  const offset = (page - 1) * limit;

  const routes = db
    .prepare("SELECT * FROM routes LIMIT ? OFFSET ?")
    .all(limit, offset);
  const total = db.prepare("SELECT COUNT(*) as count FROM routes").get().count;

  res.json({
    page,
    totalPages: Math.ceil(total / limit),
    routes,
  });
});

app.patch("/routes/:id/complete", (req, res) => {
  const { id } = req.params;
  const route = db.prepare("SELECT * FROM routes WHERE id = ?").get(id);
  if (!route) return res.status(404).json({ message: "Route not found" });

  if (route.driverId) {
    db.prepare(
      "INSERT INTO driver_history (driverId, routeId) VALUES (?, ?)"
    ).run(route.driverId, route.id);

    db.prepare(
      "UPDATE drivers SET currentRoute = NULL, availability = 1 WHERE id = ?"
    ).run(route.driverId);
  }

  db.prepare("UPDATE routes SET status = 'completed' WHERE id = ?").run(id);

  const updated = db.prepare("SELECT * FROM routes WHERE id = ?").get(id);
  res.json(updated);
});

app.listen(3000, () => console.log("Server running on http://localhost:3000"));
