from abc import ABC, abstractmethod
from pathlib import Path
from typing import List

class Document:
    """Represents a document to be ingested."""
    def __init__(self, content: str, metadata: dict):
        self.content = content
        self.metadata = metadata

class Chunk(Document):
    """Represents a chunk of a document."""
    pass

class Parser(ABC):
    """Abstract base class for a document parser."""
    @abstractmethod
    def parse(self, file_path: Path) -> Document:
        """Parses a file and returns a document."""
        pass

class MarkdownParser(Parser):
    """A parser for Markdown files."""
    def parse(self, file_path: Path) -> Document:
        """Parses a Markdown file."""
        print(f"Parsing Markdown file: {file_path}")
        with open(file_path, "r", encoding="utf-8") as f:
            content = f.read()
        
        metadata = {"source": str(file_path), "type": "markdown"}
        return Document(content=content, metadata=metadata)

class PDFParser(Parser):
    """A parser for PDF files."""
    def parse(self, file_path: Path) -> Document:
        """Parses a PDF file."""
        print(f"Parsing PDF file: {file_path}")
        # Placeholder content - replace with actual PDF parsing logic
        content = "This is a placeholder for the content of a PDF file."
        metadata = {"source": str(file_path), "type": "pdf"}
        return Document(content=content, metadata=metadata)

class TextSplitter:
    """Splits a document into smaller chunks."""
    def __init__(self, chunk_size: int = 1000, chunk_overlap: int = 200):
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap

    def split(self, doc: Document) -> List[Chunk]:
        """Splits a document into chunks."""
        chunks = []
        start = 0
        while start < len(doc.content):
            end = start + self.chunk_size
            chunk_content = doc.content[start:end]
            
            chunk_metadata = doc.metadata.copy()
            chunk_metadata["chunk_number"] = len(chunks) + 1

            chunks.append(Chunk(content=chunk_content, metadata=chunk_metadata))
            
            if end >= len(doc.content):
                break
            start += self.chunk_size - self.chunk_overlap
            
        return chunks

def get_parser(file_extension: str) -> Parser:
    """Returns a parser instance based on the file extension."""
    if file_extension == ".md":
        return MarkdownParser()
    elif file_extension == ".pdf":
        return PDFParser()
    else:
        raise ValueError(f"Unsupported file extension: {file_extension}")
