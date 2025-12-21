import logging
import asyncio
from typing import Optional
from openai import AsyncOpenAI
from config.settings import settings

logger = logging.getLogger(__name__)

class LLMService:
    """
    Service to handle communication with LLM providers via OpenRouter.
    Uses the OpenAI Python client with OpenRouter's API.
    """

    def __init__(self):
        self.client = AsyncOpenAI(
            api_key=settings.OPENROUTER_API_KEY,
            base_url="https://openrouter.ai/api/v1"
        )
        self.model = settings.LLM_MODEL

    async def generate_response(self, query: str, context: str, session_id: Optional[str] = None) -> str:
        """
        Generate response using the LLM with provided context
        """
        try:
            # Handle greetings and simple queries separately
            query_lower = query.lower().strip()
            if query_lower in ["hi", "hello", "hey", "greetings", "help", "start"]:
                greeting_message = """
                Hello! I'm your AI assistant for the Physical AI and Humanoid Robotics textbook.
                I can help you understand concepts about ROS 2, navigation systems, digital twins,
                AI integration, and humanoid robotics. Ask me about specific topics from the textbook
                or select text on the page to get detailed explanations.

                For example, you can ask:
                - "What is ROS 2?"
                - "Explain the Nav2 stack"
                - "How do humanoid robots maintain balance?"
                - Or select text on the page and click the 'Ask AI' tooltip to get explanations
                """
                return greeting_message.strip()

            # Check if this is a highlighted text explanation request
            if "Explain this:" in query:
                # This is from the highlight feature, use the extracted text as context
                selected_text = query.replace("Explain this:", "").strip()
                system_message = """
                You are an AI assistant for a Physical AI and Humanoid Robotics textbook.
                Provide a detailed explanation of the following selected text from the textbook:

                {selected_text}

                Explain the concepts clearly, break down complex ideas, and relate them to the broader context of Physical AI and Humanoid Robotics where relevant.
                """

                messages = [
                    {"role": "system", "content": system_message.format(selected_text=selected_text)},
                    {"role": "user", "content": f"Please explain the following text in detail: {selected_text}"}
                ]
            else:
                # Regular query with RAG context
                if context.strip():
                    # Check if the context contains relevant information for the query
                    context_lower = context.lower()
                    query_lower = query.lower()

                    # If context seems relevant, use it; otherwise, provide appropriate response
                    if any(keyword in context_lower for keyword in query_lower.split() if len(keyword) > 3):
                        # Context appears to contain relevant information
                        system_message = """
                        You are an AI assistant specialized in Physical AI and Humanoid Robotics.
                        Answer the user's question using ONLY the information provided in the following context.
                        Do not make up information or use general knowledge.

                        Context from the Physical AI and Humanoid Robotics textbook:
                        {context}

                        Instructions:
                        1. Base your answer strictly on the provided context
                        2. Quote relevant parts when possible
                        3. Provide accurate, detailed, and helpful answers specific to robotics
                        4. When relevant, connect concepts to Physical AI and Humanoid Robotics applications
                        5. If the context partially answers the question, provide what information is available
                        """

                        messages = [
                            {"role": "system", "content": system_message.format(context=context)},
                            {"role": "user", "content": query}
                        ]
                    else:
                        # Context exists but doesn't seem to answer the specific question
                        system_message = """
                        You are an AI assistant for the Physical AI and Humanoid Robotics textbook.
                        The retrieved context contains information from the textbook, but may not directly answer the specific question asked.
                        Use the context to provide the most relevant information possible, and if the context doesn't contain
                        the specific answer, acknowledge this limitation.

                        Retrieved context:
                        {context}

                        User's question: {query}

                        Instructions:
                        1. If the context contains relevant information, provide it
                        2. If the context doesn't directly answer the question, explain what information is available
                        3. Don't make up information not in the context
                        4. Suggest related topics from the context if applicable
                        """

                        messages = [
                            {"role": "system", "content": system_message.format(context=context, query=query)},
                            {"role": "user", "content": f"Please answer the question '{query}' based on the provided context, or explain what relevant information is available if the exact question cannot be answered."}
                        ]
                else:
                    # No context found, inform the user
                    return "I couldn't find relevant information in the textbook to answer your question. Please try asking about specific topics from the Physical AI and Humanoid Robotics content."

            # Call the LLM
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=0.3,  # Lower temperature for more consistent, factual responses
                max_tokens=1000,
                timeout=30.0
            )

            # Extract and return the response
            return response.choices[0].message.content

        except Exception as e:
            logger.error(f"Error generating response from LLM: {e}")
            raise

    async def generate_explanation(self, selected_text: str) -> str:
        """
        Generate an explanation for selected text
        """
        try:
            system_message = """
            You are an AI assistant for a Physical AI and Humanoid Robotics textbook.
            Provide a clear, simplified explanation of the following selected text.
            Break down complex concepts into understandable parts.
            """

            messages = [
                {"role": "system", "content": system_message},
                {"role": "user", "content": f"Please explain this text in simple terms:\n\n{selected_text}"}
            ]

            response = await self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=0.7,
                max_tokens=800,
                timeout=30.0
            )

            return response.choices[0].message.content

        except Exception as e:
            logger.error(f"Error generating explanation: {e}")
            raise

    async def generate_simplified_explanation(self, selected_text: str) -> str:
        """
        Generate a simplified explanation for selected text
        """
        try:
            system_message = """
            You are an AI assistant for a Physical AI and Humanoid Robotics textbook.
            Provide a simplified explanation of the following text that is easy to understand.
            Focus on the core concepts and avoid technical jargon where possible.
            """

            messages = [
                {"role": "system", "content": system_message},
                {"role": "user", "content": f"Please simplify this text:\n\n{selected_text}"}
            ]

            response = await self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=0.7,
                max_tokens=600,
                timeout=30.0
            )

            return response.choices[0].message.content

        except Exception as e:
            logger.error(f"Error generating simplified explanation: {e}")
            raise