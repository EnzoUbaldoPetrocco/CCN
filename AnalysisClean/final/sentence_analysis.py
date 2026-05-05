# =============================
# CONTENT-BASED SENTENCE ANALYSIS
# =============================

# Install required packages (run once)
# pip install sentence-transformers gensim scikit-learn keybert umap-learn nltk

import numpy as np
from sentence_transformers import SentenceTransformer
from sklearn.feature_extraction.text import CountVectorizer
from gensim.models.ldamodel import LdaModel
import gensim
from keybert import KeyBERT
from sklearn.metrics.pairwise import cosine_similarity
from sklearn.cluster import KMeans
import umap

# Optional: preprocessing
import re
import nltk
nltk.download('stopwords')
from nltk.corpus import stopwords
stop_words = set(stopwords.words('english'))

def preprocess(sentence):
    sentence = sentence.lower()
    sentence = re.sub(r'[^\w\s]', '', sentence)  # remove punctuation
    tokens = [w for w in sentence.split() if w not in stop_words]
    return ' '.join(tokens)

filename = "phrases.txt"
sentences = []

with open(filename, "r", encoding="utf-8") as f:
    lines = f.readlines()

# Combine every two lines into a single sentence
for i in range(0, len(lines), 2):
    # Remove leading/trailing whitespace and join the pair
    sentence = lines[i].strip()
    if i + 1 < len(lines):
        sentence += " " + lines[i + 1].strip()
    sentences.append(sentence)

# Check
print(f"Loaded {len(sentences)} sentences")
for s in sentences[:5]:
    print("-", s)

preprocessed = [preprocess(s) for s in sentences]

# -----------------------------
# 2. Compute sentence embeddings
# -----------------------------
embedding_model = SentenceTransformer('all-MiniLM-L6-v2')  # fast, small model
embeddings = embedding_model.encode(preprocessed)

# -----------------------------
# 3. Topic modeling (LDA)
# -----------------------------
vectorizer = CountVectorizer()
X = vectorizer.fit_transform(preprocessed)
corpus = gensim.matutils.Sparse2Corpus(X, documents_columns=False)
id2word = dict((v, k) for k, v in vectorizer.vocabulary_.items())
lda = LdaModel(corpus=corpus, num_topics=3, id2word=id2word, random_state=42)

# Get topic distribution for each sentence
topic_distributions = []
for bow in corpus:
    topic_distributions.append([t[1] for t in lda.get_document_topics(bow, minimum_probability=0)])
topic_distributions = np.array(topic_distributions)

# -----------------------------
# 4. Keyword extraction (KeyBERT)
# -----------------------------
kw_model = KeyBERT()
keywords_per_sentence = [kw_model.extract_keywords(s, keyphrase_ngram_range=(1,2), top_n=3) for s in preprocessed]

# -----------------------------
# 5. Semantic similarity and clustering
# -----------------------------
sim_matrix = cosine_similarity(embeddings)

# UMAP for visualization
reducer = umap.UMAP(n_neighbors=5, min_dist=0.3, metric='cosine', random_state=42)
embedding_2d = reducer.fit_transform(embeddings)

# KMeans clustering
num_clusters = 3
kmeans = KMeans(n_clusters=num_clusters, random_state=42)
labels = kmeans.fit_predict(embeddings)

# -----------------------------
# 6. Display / Output
# -----------------------------
for i, sentence in enumerate(sentences):
    print(f"Sentence: {sentence}")
    print(f"Preprocessed: {preprocessed[i]}")
    print(f"Topic Distribution: {topic_distributions[i]}")
    print(f"Keywords: {keywords_per_sentence[i]}")
    print(f"Cluster: {labels[i]}")
    print('-'*50)

print("2D embeddings for visualization (UMAP):")
print(embedding_2d)

import matplotlib.pyplot as plt

# embedding_2d: shape (num_sentences, 2)
# labels: cluster labels from KMeans (or any other clustering)

plt.figure(figsize=(8,6))
scatter = plt.scatter(
    embedding_2d[:,0],
    embedding_2d[:,1],
    c=labels,          # color by cluster
    cmap='tab10',      # color map
    s=50,              # marker size
    alpha=0.8
)
plt.title("UMAP Visualization of Sentence Embeddings")
plt.xlabel("UMAP 1")
plt.ylabel("UMAP 2")
plt.colorbar(scatter, label='Cluster')
plt.show()
