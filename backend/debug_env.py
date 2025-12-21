from config.settings import settings
import os

print("=== Environment Variables Debug ===")
print(f"OPENROUTER_API_KEY set: {bool(settings.OPENROUTER_API_KEY and settings.OPENROUTER_API_KEY != '')}")
print(f"QDRANT_URL set: {bool(settings.QDRANT_URL and settings.QDRANT_URL != '')}")
print(f"QDRANT_API_KEY set: {bool(settings.QDRANT_API_KEY and settings.QDRANT_API_KEY != '')}")
print(f"DATABASE_URL set: {bool(settings.DATABASE_URL and settings.DATABASE_URL != '')}")

print("\n=== Raw Environment Variables ===")
print(f"OPENROUTER_API_KEY in os.environ: {'OPENROUTER_API_KEY' in os.environ}")
print(f"QDRANT_URL in os.environ: {'QDRANT_URL' in os.environ}")
print(f"QDRANT_API_KEY in os.environ: {'QDRANT_API_KEY' in os.environ}")
print(f"DATABASE_URL in os.environ: {'DATABASE_URL' in os.environ}")

print("\n=== Settings object attributes ===")
try:
    print(f"settings.OPENROUTER_API_KEY: {repr(settings.OPENROUTER_API_KEY)}")
except:
    print("settings.OPENROUTER_API_KEY: NOT FOUND")

try:
    print(f"settings.QDRANT_URL: {repr(settings.QDRANT_URL)}")
except:
    print("settings.QDRANT_URL: NOT FOUND")

try:
    print(f"settings.QDRANT_API_KEY: {repr(settings.QDRANT_API_KEY)}")
except:
    print("settings.QDRANT_API_KEY: NOT FOUND")

try:
    print(f"settings.DATABASE_URL: {repr(settings.DATABASE_URL)}")
except:
    print("settings.DATABASE_URL: NOT FOUND")