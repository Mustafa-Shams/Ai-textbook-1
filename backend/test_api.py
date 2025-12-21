#!/usr/bin/env python3
"""
Test script to verify the RAG API is working properly
"""
import requests
import json

def test_api():
    base_url = "http://localhost:8000"

    # Test health endpoint
    print("Testing health endpoint...")
    try:
        response = requests.get(f"{base_url}/health")
        print(f"Health check: {response.status_code} - {response.json()}")
    except Exception as e:
        print(f"Health check failed: {e}")

    # Test chat endpoint
    print("\nTesting chat endpoint...")
    try:
        chat_data = {
            "query": "What is Physical AI and Humanoid Robotics about?",
            "selected_text": None,
            "session_id": "test-session-123"
        }
        response = requests.post(f"{base_url}/chat", json=chat_data, headers={"Content-Type": "application/json"})
        print(f"Chat response: {response.status_code}")
        if response.status_code == 200:
            result = response.json()
            print(f"Response length: {len(result.get('response', ''))} characters")
            print(f"Sources found: {len(result.get('sources', []))}")
            print(f"Session ID: {result.get('session_id')}")
        else:
            print(f"Error response: {response.text}")
    except Exception as e:
        print(f"Chat test failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_api()