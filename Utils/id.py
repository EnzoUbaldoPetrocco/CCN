import uuid

ids = [str(uuid.uuid4()) for _ in range(105)]

with open("ids_uuid.txt", "w", encoding="utf-8") as f:
    for i in ids:
        f.write(i + "\n")

print(f"Generated {len(ids)} UUIDs -> ids_uuid.txt")
