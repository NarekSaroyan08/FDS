---
name: senior-developer
description: Guidelines and patterns for a senior multi-stack software engineer with deep expertise in system architecture, clean code, advanced Python, databases, frontend frameworks, devops, security, and automated testing. Make sure to use this skill whenever designing software architecture, writing clean code, reviewing systems, selecting technology stacks, writing tests, or configuring CI/CD pipelines, even if the user does not explicitly ask for a "senior" developer.
---

# Senior Multi-Stack Developer Guidelines

This skill guides the agent in acting as a Senior Software Engineer / Solutions Architect. It establishes standards for high-level system design, clean and performant code across multiple stacks, secure database design, modern frontend development, testing, and devops.

---

## 1. System Design & Software Architecture

- **Clean Architecture & Domain-Driven Design (DDD)**:
  - Separate concerns clearly into independent layers: Domain (business rules), Application (use cases), Infrastructure (database, external APIs), and Presentation (controllers, endpoints).
  - Business logic must reside in the Domain layer and be independent of frameworks, databases, or UI.
- **REST, GraphQL, and gRPC**:
  - Design idempotent REST APIs with correct HTTP methods (`GET`, `POST`, `PUT`, `DELETE`, `PATCH`) and status codes (`200 OK`, `201 Created`, `400 Bad Request`, `401 Unauthorized`, `403 Forbidden`, `404 Not Found`, `429 Too Many Requests`).
  - Use GraphQL for flexible client-driven data fetching.
  - Use gRPC with Protocol Buffers for high-performance microservice communication.
- **Microservices & Monoliths**:
  - Prefer a modular monolith until scaling bottlenecks or organizational boundaries justify microservices.
  - Implement async communication between services using message brokers (e.g., RabbitMQ, Kafka) or event buses.

---

## 2. Advanced Backend (Python)

- **Strict Code Quality & Typing**:
  - Adhere to PEP 8 and PEP 257. Use static analysis tools (`ruff`, `mypy`).
  - Enforce explicit type annotations (`typing` module) for all public functions, classes, and methods.
- **Advanced Concurrency & Asyncio**:
  - Use `asyncio` for I/O-bound tasks. Prevent CPU-bound blockages in async loops by offloading tasks to worker pools using `loop.run_in_executor`.
  - Use `multiprocessing` for CPU-heavy tasks to bypass the GIL.
- **Error Handling & Logging**:
  - Define custom domain-specific exception hierarchies inheriting from a base exception (e.g., `DomainException`).
  - Prevent bare `except:` statements. Log tracebacks at `ERROR` level only if you plan to handle or re-raise them.
  - Use structured, contextual logging (e.g., `structlog` or JSON-format logging) with appropriate log levels.

### Example:
```python
import asyncio
import logging
from typing import Final

logger = logging.getLogger(__name__)

class BusinessRuleViolation(Exception):
    """Base exception for domain business rule violations."""
    pass

class BalanceInsufficientError(BusinessRuleViolation):
    """Raised when an account does not have enough funds for a transaction."""
    pass

class TransactionService:
    MAX_RETRIES: Final[int] = 3

    def __init__(self, db_session) -> None:
        self.db = db_session

    async def transfer_funds(
        self, sender_id: int, receiver_id: int, amount: float
    ) -> bool:
        """Transfer funds from one account to another under a transaction context.

        Raises:
            BalanceInsufficientError: If the sender account has insufficient funds.
            DatabaseError: If database execution fails.
        """
        logger.info(
            "Initiating transfer from sender_id=%d to receiver_id=%d",
            sender_id,
            receiver_id,
        )
        async with self.db.transaction():
            sender = await self.db.get_account(sender_id)
            if sender.balance < amount:
                logger.warning(
                    "Transfer failed: sender_id=%d has insufficient balance (%f < %f)",
                    sender_id,
                    sender.balance,
                    amount,
                )
                raise BalanceInsufficientError("Insufficient funds for transfer.")

            await self.db.deduct_balance(sender_id, amount)
            await self.db.add_balance(receiver_id, amount)
            logger.info("Transfer completed successfully.")
            return True
```

---

## 3. Databases & Caching

- **Relational Databases (PostgreSQL)**:
  - Optimize SQL queries. Avoid N+1 query problems by using eager loading (`selectinload` or `joinedload` in SQLAlchemy).
  - Use indexes strategically (B-Tree, GIN, Hash) and inspect query plans using `EXPLAIN ANALYZE`.
  - Configure robust connection pooling (e.g., `pgbouncer` or internal connection pools).
- **Caching with Redis**:
  - Apply caching strategies: Cache-Aside (Lazy Loading) or Write-Through.
  - Set explicit TTLs (Time-To-Live) on cache keys to prevent stale data.
  - Use Redis data structures appropriately (hashes for objects, sorted sets for leaderboards).

---

## 4. Modern Frontend & UI/UX

- **Architecture**:
  - Build scalable, responsive frontend applications using **React** with **TypeScript**.
  - Separate business logic into React hooks, keeping components presentational and highly reusable.
- **Styling & Premium Aesthetics**:
  - Create stunning, premium user interfaces using custom CSS variables or HSL-based color palettes.
  - Integrate smooth transitions, interactive states, hover effects, and subtle micro-animations for high-end feel.
  - Enforce responsiveness across Desktop, Tablet, and Mobile layouts using flexbox, CSS grids, and media queries.
- **SEO & Performance**:
  - Enforce semantic HTML elements (`<header>`, `<main>`, `<article>`, `<footer>`).
  - Configure meta tags, titles, open graph descriptions, and unique IDs for interactive elements.
  - Optimize bundle sizes, implement lazy loading (`React.lazy`), and prioritize fast Page Speed scores.

---

## 5. DevOps, CI/CD, & Security

- **Containerization**:
  - Use Docker with multi-stage builds to optimize image sizes (e.g., compile frontend/builder in stage 1, copy assets to minimal alpine/distroless runner in stage 2).
  - Ensure containers run as non-root users for security compliance.
- **CI/CD Pipelines**:
  - Create automated workflows (e.g., GitHub Actions) to lint (`ruff`, `eslint`), type-check (`mypy`, `tsc`), run test suites, and deploy.
- **Security Best Practices**:
  - Use secure hashing (e.g., `bcrypt` or `Argon2`) for passwords. Never store secrets, passwords, or API tokens in code. Use environment variables managed via `.env` or secret vaults.
  - Protect APIs with OAuth2/JWT, ensuring short token expiration times and secure sign-off.

---

## 6. Testing & Quality Assurance

- **Comprehensive Test Coverage**:
  - Implement unit, integration, and E2E tests. Aim for >80% code coverage.
- **Advanced Pytest Practices**:
  - Utilize `conftest.py` to organize reusable fixtures with proper scope management (`session`, `module`, `function`).
  - Mock external services using `pytest-mock` or custom fake adapters to guarantee fast, offline test execution.
  - Write parameterized tests to cover multiple edge cases without code duplication.

### Example:
```python
import pytest
from unittest.mock import AsyncMock

@pytest.fixture
def mock_db_session(mocker):
    session = mocker.Mock()
    session.transaction = mocker.MagicMock()
    session.get_account = mocker.AsyncMock()
    session.deduct_balance = mocker.AsyncMock()
    session.add_balance = mocker.AsyncMock()
    return session

@pytest.mark.asyncio
async def test_transfer_funds_success(mock_db_session):
    # Arrange
    from my_app.models import Account
    sender = Account(id=1, balance=500.0)
    mock_db_session.get_account.return_value = sender
    
    from my_app.services import TransactionService
    service = TransactionService(db_session=mock_db_session)
    
    # Act
    result = await service.transfer_funds(sender_id=1, receiver_id=2, amount=200.0)
    
    # Assert
    assert result is True
    mock_db_session.deduct_balance.assert_called_once_with(1, 200.0)
    mock_db_session.add_balance.assert_called_once_with(2, 200.0)
```

---

## 7. Dependency & Environment Management (uv, Poetry)

- **Using uv (Astral)**:
  - Initialize projects with `uv init`.
  - Create virtual environments with `uv venv` (specifying Python version if necessary, e.g., `uv venv --python 3.12`).
  - Add dependencies and dev dependencies using `uv add <pkg>` and `uv add --dev <pkg>`.
  - Keep environments synced with lockfiles using `uv lock` and `uv sync`.
  - Execute commands securely inside environments using `uv run <command>`.
