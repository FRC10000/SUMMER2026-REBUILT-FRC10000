import sqlite3, json

conn = sqlite3.connect(r'C:\Users\Alvin\.local\share\mimocode\mimocode.db')
c = conn.cursor()

# Find the SmartDashboard auto path session - check if there's useful content
session_id = 'ses_0a3cfd6c2ffeo5irl3I3XRGO62'
c.execute("SELECT m.id, m.agent_id, json_extract(m.data, '$.role') as role, m.time_created FROM message m WHERE m.session_id = ? ORDER BY m.time_created", (session_id,))
msgs = c.fetchall()
print(f"=== SmartDashboard session ({len(msgs)} messages) ===")
for m in msgs:
    print(f"  msg {m[0][:20]}... agent={m[1]} role={m[2]} time={m[3]}")

# Get last 3 messages
if msgs:
    for m in msgs[-3:]:
        c.execute("SELECT substr(p.data, 1, 500) FROM part p WHERE p.message_id = ? ORDER BY p.time_created", (m[0],))
        parts = c.fetchall()
        for p in parts:
            print(f"\n  Part preview: {p[0][:400]}")

# Check the most recent project session (ses_0a3cfd695ffeq8Tu85pbFG5Ut8 = Auto Dream = current)
session_id2 = 'ses_0a3cfd695ffeq8Tu85pbFG5Ut8'
c.execute("SELECT count(*) FROM message m WHERE m.session_id = ?", (session_id2,))
count = c.fetchone()[0]
print(f"\n=== Current Auto Dream session: {count} messages ===")

# Check for any sessions with tasks
c.execute("SELECT t.id, t.session_id, t.status, substr(t.summary, 1, 80) FROM task t ORDER BY t.created_at DESC LIMIT 10")
tasks = c.fetchall()
print(f"\n=== Tasks in DB ({len(tasks)}) ===")
for t in tasks:
    print(f"  {t[0]} | session={t[1][:20]}... | status={t[2]} | {t[3]}")

# Check recent sessions not in memory files
# ses_0aa3d5fdaffeB0Dmn1vABNcRff (nimble-cactus, New session)
# ses_0aa54e4bbffetoZ3fII5r4axXT (eager-otter, New session)
for sid in ['ses_0aa3d5fdaffeB0Dmn1vABNcRff', 'ses_0aa54e4bbffetoZ3fII5r4axXT']:
    c.execute("SELECT count(*) FROM message m WHERE m.session_id = ?", (sid,))
    cnt = c.fetchone()[0]
    c.execute("SELECT title FROM session WHERE id = ?", (sid,))
    title = c.fetchone()[0]
    print(f"\n=== {sid} ({title}): {cnt} messages ===")
    if cnt > 0:
        c.execute("SELECT substr(p.data, 1, 300) FROM part p JOIN message m ON p.message_id = m.id WHERE m.session_id = ? ORDER BY m.time_created LIMIT 3", (sid,))
        for p in c.fetchall():
            print(f"  {p[0][:250]}")

# Check ses_0acf87317ffedVbC20WqV5UGsY (autoAim oscillation) - does it have the limelight primary target work?
session_id3 = 'ses_0acf87317ffedVbC20WqV5UGsY'
c.execute("SELECT count(*) FROM message m WHERE m.session_id = ?", (session_id3,))
cnt = c.fetchone()[0]
print(f"\n=== autoAim oscillation session: {cnt} messages ===")

# Check for LimelightHelpers references
c.execute("""SELECT DISTINCT p.session_id, substr(p.data, 1, 200) FROM part p 
             WHERE p.data LIKE '%LimelightHelpers%' 
             ORDER BY p.time_created DESC LIMIT 5""")
for r in c.fetchall():
    print(f"\n  LimelightHelpers in session {r[0][:20]}...: {r[1][:150]}")

conn.close()
