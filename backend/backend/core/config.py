import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class Settings:
    """
    Application settings loaded from the environment.
    """
    # Gemini
    GEMINI_API_KEY: str = os.getenv("GEMINI_API_KEY")

    # Qdrant
    QDRANT_URL: str = os.getenv("QDRANT_URL", "http://localhost:6333")
    QDRANT_API_KEY: str | None = os.getenv("QDRANT_API_KEY")

    # Database
    DATABASE_URL: str = os.getenv("DATABASE_URL")

# Instantiate settings to be used throughout the application
settings = Settings()

# Validate that essential settings are present
if not settings.GEMINI_API_KEY:
    raise ValueError("GEMINI_API_KEY must be set in the .env file.")
if not settings.DATABASE_URL:
    raise ValueError("DATABASE_URL must be set in the .env file.")

