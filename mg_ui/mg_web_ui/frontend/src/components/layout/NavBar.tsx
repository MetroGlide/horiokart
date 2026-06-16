import { NavLink } from "react-router-dom";

const links = [
  { to: "/", label: "TOP" },
  { to: "/waypoint", label: "Waypoint Nav" },
  { to: "/slam", label: "SLAM Toolbox" },
  { to: "/slam-gnss-2d", label: "SLAM-GNSS-2D" },
  { to: "/system", label: "System" },
];

interface NavBarProps {
  onSettingClick: () => void;
}

export default function NavBar({ onSettingClick }: NavBarProps) {
  return (
    <nav className="flex items-center bg-gray-800 border-b border-gray-700">
      <div className="flex">
        {links.map(({ to, label }) => (
          <NavLink
            key={to}
            to={to}
            end={to === "/"}
            className={({ isActive }) =>
              `px-5 py-3 text-sm font-medium transition-colors ${
                isActive
                  ? "text-blue-400 border-b-2 border-blue-400"
                  : "text-gray-400 hover:text-white"
              }`
            }
          >
            {label}
          </NavLink>
        ))}
      </div>
      <div className="ml-auto pr-3">
        <button
          onClick={onSettingClick}
          className="p-2 text-gray-400 hover:text-white rounded transition-colors"
          title="Settings"
        >
          <svg
            className="w-5 h-5"
            fill="none"
            viewBox="0 0 24 24"
            stroke="currentColor"
          >
            <path
              strokeLinecap="round"
              strokeLinejoin="round"
              strokeWidth={2}
              d="M10.325 4.317c.426-1.756 2.924-1.756 3.35 0a1.724 1.724 0 002.573 1.066c1.543-.94 3.31.826 2.37 2.37a1.724 1.724 0 001.065 2.572c1.756.426 1.756 2.924 0 3.35a1.724 1.724 0 00-1.066 2.573c.94 1.543-.826 3.31-2.37 2.37a1.724 1.724 0 00-2.572 1.065c-.426 1.756-2.924 1.756-3.35 0a1.724 1.724 0 00-2.573-1.066c-1.543.94-3.31-.826-2.37-2.37a1.724 1.724 0 00-1.065-2.572c-1.756-.426-1.756-2.924 0-3.35a1.724 1.724 0 001.066-2.573c-.94-1.543.826-3.31 2.37-2.37.996.608 2.296.07 2.572-1.065z"
            />
            <path
              strokeLinecap="round"
              strokeLinejoin="round"
              strokeWidth={2}
              d="M15 12a3 3 0 11-6 0 3 3 0 016 0z"
            />
          </svg>
        </button>
      </div>
    </nav>
  );
}
