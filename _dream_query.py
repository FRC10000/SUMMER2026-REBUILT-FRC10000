import sqlite3, json

conn = sqlite3.connect(r'C:\Users\Alvin\.local\share\mimocode\mimocode.db')
c = conn.cursor()

# Check schema
c.execute("SELECT name FROM sqlite_master WHERE type='table'")
tables = [r[0] for r in c.fetchall()]
print("Tables:", tables)

for t in tables:
    c.execute(f"PRAGMA table_info({t})")
    cols = [r[1] for r in c.fetchall()]
    print(f"\n{t} columns: {cols}")

# List sessions
c.execute("SELECT * FROM session ORDER BY time_created DESC LIMIT 20")
rows = c.fetchall()
for r in rows:
    print(r)

conn.close()
