import sqlite3, json

conn = sqlite3.connect(r'C:\Users\Alvin\.local\share\mimocode\mimocode.db')
c = conn.cursor()

# Get all messages from the autoAim oscillation session that had tasks completed
# ses_0acf87317ffedVbC20WqV5UGsY had tasks T1-T7 all "done"
session_id = 'ses_0acf87317ffedVbC20WqV5UGsY'
c.execute("""SELECT m.id, json_extract(m.data, '$.role') as role, m.time_created 
             FROM message m WHERE m.session_id = ? AND json_extract(m.data, '$.role') = 'assistant'
             ORDER BY m.time_created""", (session_id,))
msgs = c.fetchall()
print(f"=== autoAim oscillation session: {len(msgs)} assistant messages ===")

# Get the last few assistant messages to see what was done
for m in msgs[-5:]:
    c.execute("""SELECT json_extract(p.data, '$.type') as ptype, substr(p.data, 1, 600) 
                 FROM part p WHERE p.message_id = ? ORDER BY p.time_created""", (m[0],))
    parts = c.fetchall()
    for p in parts:
        if p[0] == 'text':
            print(f"\n--- Text part ---")
            print(p[1][:500])
        elif p[0] == 'tool':
            print(f"\n--- Tool call ---")
            print(p[1][:500])

# Check the ses_0aa54e4bbffetoZ3fII5r4axXT session (eager-otter) more carefully
session_id2 = 'ses_0aa54e4bbffetoZ3fII5r4axXT'
c.execute("""SELECT m.id, json_extract(m.data, '$.role') as role, m.time_created 
             FROM message m WHERE m.session_id = ? ORDER BY m.time_created""", (session_id2,))
msgs2 = c.fetchall()
print(f"\n\n=== eager-otter session: {len(msgs2)} messages ===")
for m in msgs2:
    c.execute("""SELECT json_extract(p.data, '$.type') as ptype, substr(p.data, 1, 600) 
                 FROM part p WHERE p.message_id = ? ORDER BY p.time_created""", (m[0],))
    parts = c.fetchall()
    for p in parts:
        if p[0] == 'text':
            role = m[1]
            print(f"\n--- {role} text ---")
            print(p[1][:500])

# Check the ses_0a3cfd6c2ffeo5irl3I3XRGO62 session (SmartDashboard) - full content
session_id3 = 'ses_0a3cfd6c2ffeo5irl3I3XRGO62'
c.execute("""SELECT m.id, json_extract(m.data, '$.role') as role, m.time_created 
             FROM message m WHERE m.session_id = ? ORDER BY m.time_created""", (session_id3,))
msgs3 = c.fetchall()
print(f"\n\n=== SmartDashboard session: {len(msgs3)} messages ===")
for m in msgs3:
    c.execute("""SELECT json_extract(p.data, '$.type') as ptype, substr(p.data, 1, 600) 
                 FROM part p WHERE p.message_id = ? ORDER BY p.time_created""", (m[0],))
    parts = c.fetchall()
    for p in parts:
        if p[0] == 'text':
            role = m[1]
            print(f"\n--- {role} text ---")
            print(p[1][:500])

conn.close()
