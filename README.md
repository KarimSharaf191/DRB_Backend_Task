
# 🚖 Route & Driver Management API

A simple Express + SQLite API for managing drivers, assigning them to routes, tracking schedules, and keeping driver history.

---

## ⚙️ Setup Instructions

1. Clone the repository:
   ```bash
   git clone <your-repo-url>
   cd <your-repo-folder>
   ```
then make sure your json 
2. Initialize Node.js project:
   ```bash
   npm init -y
```
3. Edit package.json and add (uploaded an example json):
"type": "module"

4. Install dependencies:
```bash
npm install
```
5. Start the server:
```bash
node server.js
```

The server will run at: http://localhost:3000
###########

📝 Assumptions Made

- Drivers can only have one active route at a time (currentRoute).

- A route is created as unassigned unless an available driver is found.

- Drivers marked available = 1 are considered ready for auto-assignment.

- A driver cannot be set to off-duty (availability 0) while assigned to a route.

- When a route is completed, the driver is automatically freed (currentRoute = NULL, availability = 1) and a record is added to driver_history
Each driver has a unique id (primary key).

#########################
✨ Features Implemented

- Add new drivers (POST /drivers)

- Retrieve all drivers (GET /drivers)

- Update driver availability (PATCH /drivers/:id/availability)

- Create new routes (POST /routes)

- Retrieve paginated routes (GET /routes)

- Complete a route (PATCH /routes/:id/complete)

- View driver history (GET /drivers/:id/history)

- View full schedule with driver assignments (GET /schedule)
   
📌 Example Commands ( use CMD)

# Add a driver
```bash
curl -X POST http://localhost:3000/drivers \
  -H "Content-Type: application/json" \
  -d "{\"id\":\"D1\",\"name\":\"Karim\",\"licenseType\":\"A\",\"availability\":1}"
```


# Add a route
```bash
curl -X POST http://localhost:3000/routes \
  -H "Content-Type: application/json" \
  -d "{\"startLocation\":\"Cairo\",\"endLocation\":\"Alexandria\",\"distance\":220,\"estimatedTime\":\"3h\"}"
```

# Mark route complete
```bash
curl -X PATCH http://localhost:3000/routes/1/complete
```


# Update driver availability
```bash
curl -X PATCH http://localhost:3000/drivers/D1/availability \
  -H "Content-Type: application/json" \
  -d "{\"availability\":1}"
```
# Get all drivers
```bash


curl http://localhost:3000/drivers
```


# Get all routes (paginated, default page=1, limit=5)
```bash

curl http://localhost:3000/routes
```

# Get all routes (page 2, limit 3)
```bash

curl http://localhost:3000/schedule
```

# Get driver history
```bash

curl http://localhost:3000/drivers/D1/history
```

# Mark route complete
```bash

curl -X PATCH http://localhost:3000/routes/1/complete
```


# Update driver availability
```bash

curl -X PATCH http://localhost:3000/drivers/D1/availability ^
  -H "Content-Type: application/json" ^
  -d "{\"availability\":1}"
  ```








   
   
