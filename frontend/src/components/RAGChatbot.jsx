import React, { useState } from 'react';


export default function RAGChatbot() {
  const [question, setQuestion] = useState("");
  const [answer, setAnswer] = useState("");
  const [citations, setCitations] = useState([]);
  const [loading, setLoading] = useState(false);

  const askQuestion = async () => {
    setLoading(true);
    setAnswer("");
    setCitations([]);

    const selectedText = window.getSelection().toString();

    const response = await fetch("http://localhost:8000/query", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        question,
        selected_text: selectedText || null
      }),
    });

    const data = await response.json();
    setAnswer(data.answer);
    setCitations(data.citations || []);
    setLoading(false);
  };

  return (
    <div style={{
      border: "1px solid #ddd",
      borderRadius: "12px",
      padding: "16px",
      marginTop: "24px",
      background: "#fafafa"
    }}>
      <h3>📘 Ask the Book</h3>

      <textarea
        rows="3"
        placeholder="Ask a question about the book…"
        value={question}
        onChange={(e) => setQuestion(e.target.value)}
        style={{ width: "100%", padding: "8px" }}
      />

      <button
        onClick={askQuestion}
        disabled={loading}
        style={{ marginTop: "8px" }}
      >
        {loading ? "Thinking…" : "Ask"}
      </button>

      {answer && (
        <div style={{ marginTop: "16px" }}>
          <h4>Answer</h4>
          <p>{answer}</p>

          {citations.length > 0 && (
            <>
              <h5>Sources</h5>
              <ul>
                {citations.map((c, i) => (
                  <li key={i}>{c}</li>
                ))}
              </ul>
            </>
          )}
        </div>
      )}
    </div>
  );
}
