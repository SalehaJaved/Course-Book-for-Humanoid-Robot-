import asyncio
import uuid
from pathlib import Path

import google.generativeai as genai
from qdrant_client import QdrantClient, models

from backend.core.config import settings
from backend.ingestion.parser import TextSplitter, get_parser

# Configure the Gemini client
genai.configure(api_key=settings.GEMINI_API_KEY)

# Configure the Qdrant client
qdrant_client = QdrantClient(url=settings.QDRANT_URL, api_key=settings.QDRANT_API_KEY)

COLLECTION_NAME = "physical_ai_book"

async def ingest_documents():
    """
    Orchestrates the ingestion of documents from the source directory
    into the vector store.
    """
    source_dir = Path("data/source")
    if not source_dir.exists():
        print(f"Source directory not found: {source_dir}")
        return

    print("--- Starting document ingestion ---")

    # 1. Setup Qdrant collection
    try:
        collection_exists = qdrant_client.get_collection(collection_name=COLLECTION_NAME)
        if collection_exists:
            print(f"Qdrant collection '{COLLECTION_NAME}' already exists.")
        else:
            qdrant_client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE),
            )
            print(f"Qdrant collection '{COLLECTION_NAME}' created.")
    except Exception as e:
        # If the collection does not exist, Qdrant throws an error.
        # We can create it in the except block.
        try:
            qdrant_client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE),
            )
            print(f"Qdrant collection '{COLLECTION_NAME}' created.")
        except Exception as e2:
            print(f"Could not create Qdrant collection: {e2}")
            return

    # 2. Initialize parser and text splitter
    text_splitter = TextSplitter(chunk_size=500, chunk_overlap=100)
    
    # 3. Process each file
    files_to_process = list(source_dir.glob("**/*"))
    print(f"Found {len(files_to_process)} files to process.")

    for file_path in files_to_process:
        if file_path.is_file():
            try:
                print(f"\nProcessing file: {file_path.name}")
                parser = get_parser(file_path.suffix)
                doc = parser.parse(file_path)
                
                # 4. Split document into chunks
                chunks = text_splitter.split(doc)
                print(f"  - Split into {len(chunks)} chunks.")

                # 5. Generate embeddings and prepare for upsert
                points_to_upsert = []
                for i, chunk in enumerate(chunks):
                    # Generate embedding
                    embedding_result = genai.embed_content(
                        model="models/embedding-001",
                        content=chunk.content,
                        task_type="retrieval_document"
                    )
                    
                    # Prepare Qdrant point
                    point = models.PointStruct(
                        id=str(uuid.uuid4()),
                        vector=embedding_result['embedding'],
                        payload={
                            "text": chunk.content,
                            **chunk.metadata
                        }
                    )
                    points_to_upsert.append(point)

                # 6. Upsert points to Qdrant
                if points_to_upsert:
                    qdrant_client.upsert(
                        collection_name=COLLECTION_NAME,
                        points=points_to_upsert,
                        wait=True
                    )
                    print(f"  - Upserted {len(points_to_upsert)} points to Qdrant.")

            except ValueError as e:
                print(f"  - Skipping file due to error: {e}")
            except Exception as e:
                print(f"  - An unexpected error occurred: {e}")
    
    print("\n--- Document ingestion finished ---")

if __name__ == "__main__":
    asyncio.run(ingest_documents())
