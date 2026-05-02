import { NavLink } from "react-router-dom";

const links = [
  { to: "/", label: "TOP" },
  { to: "/waypoint", label: "Waypoint Nav" },
  { to: "/slam", label: "SLAM" },
  { to: "/utility", label: "Utility" },
];

export default function NavBar() {
  return (
    <nav className="flex bg-gray-800 border-b border-gray-700">
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
    </nav>
  );
}
